#!/usr/bin/env python3
"""ROS2 Cartesian target planner for a 5-motor Dynamixel arm.

READ-ONLY by default. Physical motion is deliberately locked until the
calibration and workspace are independently verified.
"""
import argparse
import time
import numpy as np
import dynamixel_sdk as dxl
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from spatialmath import SE3

from rosify_dynamixel_ros_pkg.robot_model import create_robot
from rosify_dynamixel_ros_pkg.motor_config import (
    MOTOR_IDS, MOTOR_LIMITS, motor_to_joint, joint_to_motor
)

PORT = '/dev/ttyUSB0'
BAUDRATE = 1000000
PROTOCOL = 2.0
TCP_OFFSET_M = 0.080
POSITION_TOL_M = 0.005
MAX_TOTAL_STEPS = 30       # hard first-test limit, each moving motor
MAX_WAYPOINT_STEPS = 5     # commanded motor change per waypoint
FEEDBACK_TOL_STEPS = 4  # NOT sufficient to verify 5-step waypoints
WAYPOINT_TIMEOUT_S = 4.0
PROFILE_VELOCITY = 5
PROFILE_ACCELERATION = 2

# DO NOT CHANGE THIS until DH zero, TCP, collision envelope, and E-stop
# have been physically verified. A near-looking photo is insufficient.
PHYSICAL_CALIBRATION_VERIFIED = False

ADDR_MODE = 11
ADDR_TORQUE = 64
ADDR_HW_ERROR = 70
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL = 116
ADDR_PRESENT = 132
ADDR_MOVING = 122
ADDR_MOVING_STATUS = 123

# Monitoring only: no changes to Dynamixel configuration in dry-run.
STABILITY_SAMPLES = 3
STABLE_STEP_BAND = 1
PROGRESS_MIN_STEPS = 1


class MoveToPointSafe(Node):
    def __init__(self, execute=False):
        super().__init__('move_to_point_safe')
        self.execute = execute
        self.completed = False
        self.busy = False
        self.robot = create_robot()
        self.robot.tool = SE3.Tz(TCP_OFFSET_M)
        self.port = dxl.PortHandler(PORT)
        self.packet = dxl.PacketHandler(PROTOCOL)
        if not self.port.openPort():
            raise RuntimeError(f'Cannot open {PORT}')
        if not self.port.setBaudRate(BAUDRATE):
            self.port.closePort()
            raise RuntimeError('Cannot set baudrate')
        self.sub = self.create_subscription(
            Float64MultiArray, '/target_point', self.on_target, 10)
        self.get_logger().info(
            f'Started; mode={"EXECUTE" if execute else "READ-ONLY DRY RUN"}; '
            f'TCP offset={TCP_OFFSET_M:.3f} m')

    def check(self, result, error, context):
        if result != dxl.COMM_SUCCESS:
            raise RuntimeError(f'{context}: {self.packet.getTxRxResult(result)}')
        if error != 0:
            raise RuntimeError(f'{context}: {self.packet.getRxPacketError(error)}')

    def read1(self, motor_id, address):
        value, result, error = self.packet.read1ByteTxRx(
            self.port, motor_id, address)
        self.check(result, error, f'read motor {motor_id} addr {address}')
        return int(value)

    def read_position(self, motor_id):
        raw, result, error = self.packet.read4ByteTxRx(
            self.port, motor_id, ADDR_PRESENT)
        self.check(result, error, f'position motor {motor_id}')
        return raw if raw < 2**31 else raw - 2**32

    def read_positions(self):
        return np.array([self.read_position(i) for i in MOTOR_IDS], dtype=int)

    def read4_signed(self, motor_id, address):
        raw, result, error = self.packet.read4ByteTxRx(
            self.port, motor_id, address)
        self.check(result, error, f'read motor {motor_id} addr {address}')
        return int(raw if raw < 2**31 else raw - 2**32)

    def read_motor_status(self):
        """Read-only snapshot of motor 1-4 status and actual goal error.

        Bit 0 is the servo's own In-Position indication; it is not
        equivalent to zero actual position error.
        """
        status = {}
        for motor_id in MOTOR_IDS[:4]:
            goal = self.read4_signed(motor_id, ADDR_GOAL)
            present = self.read_position(motor_id)
            moving = self.read1(motor_id, ADDR_MOVING)
            flags = self.read1(motor_id, ADDR_MOVING_STATUS)
            hw = self.read1(motor_id, ADDR_HW_ERROR)
            status[motor_id] = {
                'goal': goal,
                'present': present,
                'error': present - goal,
                'moving': moving,
                'in_position': bool(flags & 0x01),
                'profile_ongoing': bool(flags & 0x02),
                'following_error': bool(flags & 0x08),
                'hw_error': hw,
                'raw_moving_status': flags,
            }
        return status

    def print_motor_status(self):
        print('Motor feedback (read-only):', flush=True)
        print('ID   Goal  Present   Error  Moving  InPos  Profile  Following  HW')
        for motor_id, s in self.read_motor_status().items():
            print(f'{motor_id:2d} {s["goal"]:6d} {s["present"]:8d} '
                  f'{s["error"]:+7d} {s["moving"]:7d} '
                  f'{int(s["in_position"]):6d} '
                  f'{int(s["profile_ongoing"]):8d} '
                  f'{int(s["following_error"]):10d} '
                  f'{s["hw_error"]:3d}')
        print(flush=True)

    def write4(self, motor_id, address, value):
        result, error = self.packet.write4ByteTxRx(
            self.port, motor_id, address, int(value))
        self.check(result, error, f'write motor {motor_id} addr {address}')

    def validate_positions(self, positions, label):
        for motor_id, p in zip(MOTOR_IDS, positions):
            low, high = MOTOR_LIMITS[motor_id]
            if not low <= int(p) <= high:
                raise RuntimeError(
                    f'{label}: motor {motor_id} pos {p} outside [{low}, {high}]')

    def plan(self, target):
        current = self.read_positions()
        self.validate_positions(current, 'Actual position')
        q_current = motor_to_joint(current)
        ik = self.robot.ikine_LM(
            SE3(*target), q0=q_current,
            mask=[1, 1, 1, 0, 0, 0], joint_limits=True)
        if not ik.success or ik.q is None:
            raise RuntimeError(f'IK failed: {ik.reason}')
        q_target = np.asarray(ik.q, dtype=float).copy()
        q_target[4] = q_current[4]  # do not command motor 5
        xyz = np.asarray(self.robot.fkine(q_target).t, dtype=float).reshape(3)
        error = float(np.linalg.norm(xyz - target))
        if error > POSITION_TOL_M:
            raise RuntimeError(f'FK target error too high: {error*1000:.2f} mm')
        goal = np.array(joint_to_motor(q_target), dtype=int)
        goal[4] = current[4]
        self.validate_positions(goal, 'Goal position')
        delta = goal - current
        max_delta = int(np.max(np.abs(delta[:4])))
        # Joint-space interpolation: checks MOTOR bounds at every waypoint.
        n = max(1, int(np.ceil(max_delta / MAX_WAYPOINT_STEPS)))
        waypoints = []
        for k in range(1, n + 1):
            w = np.rint(current + (k / n) * delta).astype(int)
            w[4] = current[4]
            self.validate_positions(w, f'Waypoint {k}')
            if waypoints and np.max(np.abs(w[:4]-waypoints[-1][:4])) > MAX_WAYPOINT_STEPS:
                raise RuntimeError('Waypoint step limit exceeded')
            waypoints.append(w)
        return current, goal, q_current, q_target, xyz, error, waypoints

    def preflight_motion(self, start, goal):
        if not PHYSICAL_CALIBRATION_VERIFIED:
            raise RuntimeError('PHYSICAL_CALIBRATION_VERIFIED=False; motion locked')
        if np.max(np.abs((goal - start)[:4])) > MAX_TOTAL_STEPS:
            raise RuntimeError('Target exceeds first-test travel limit')
        # Only IDs 1-4 commanded. Motor 5 deliberately excluded.
        for i in MOTOR_IDS:
            mode = self.read1(i, ADDR_MODE)
            expected = 5 if i == 5 else 3
            if mode != expected:
                raise RuntimeError(f'Motor {i}: operating mode {mode} != {expected}')
            if self.read1(i, ADDR_HW_ERROR) != 0:
                raise RuntimeError(f'Motor {i}: hardware error flag set')
            if i != 5 and self.read1(i, ADDR_TORQUE) != 1:
                raise RuntimeError(f'Motor {i}: torque OFF (will not auto-enable)')
        fresh = self.read_positions()
        if np.max(np.abs(fresh - start)) > 3:
            raise RuntimeError('Position moved since planning; abort')
        self.validate_positions(fresh, 'Fresh position')

    def move_waypoints(self, waypoints):
        """Conservative waypoint monitor; not a collision-safe trajectory.

        Never accept a waypoint merely because its error is <= 4 steps.
        Require motion towards each command and several stable feedback
        samples. Fail closed if a loaded motor cannot settle close enough.
        This may time out on motors 2/3 (observed offsets 6-7 steps);
        do not loosen tolerances just to make execution possible.
        """
        for motor_id in MOTOR_IDS[:4]:
            self.write4(motor_id, ADDR_PROFILE_ACCEL, PROFILE_ACCELERATION)
            self.write4(motor_id, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY)

        previous_goal = self.read_positions()
        for k, waypoint in enumerate(waypoints, start=1):
            fresh = self.read_positions()
            self.validate_positions(fresh, f'Pre-waypoint {k}')
            if k > 1 and np.max(np.abs(fresh[:4] - previous_goal[:4])) > FEEDBACK_TOL_STEPS:
                raise RuntimeError(f'Drift before waypoint {k}; stop sending')

            if np.max(np.abs(waypoint[:4] - fresh[:4])) > MAX_WAYPOINT_STEPS + FEEDBACK_TOL_STEPS:
                raise RuntimeError(f'Actual-to-command jump too large at waypoint {k}')

            commanded_delta = waypoint[:4] - fresh[:4]
            previous_abs_error = np.abs(commanded_delta)
            for motor_id, pos in zip(MOTOR_IDS[:4], waypoint[:4]):
                self.write4(motor_id, ADDR_GOAL, int(pos))

            deadline = time.monotonic() + WAYPOINT_TIMEOUT_S
            stability_count = 0
            prior = None
            progress_seen = bool(np.all(previous_abs_error <= 2))
            while time.monotonic() < deadline:
                status = self.read_motor_status()
                actual = np.array([status[i]['present'] for i in MOTOR_IDS[:4]], dtype=int)
                self.validate_positions(self.read_positions(), 'Feedback')
                if any(s['hw_error'] != 0 or s['following_error'] for s in status.values()):
                    raise RuntimeError(f'Hardware/following error at waypoint {k}')
                error = np.abs(actual - waypoint[:4])
                if np.any(error < previous_abs_error):
                    progress_seen = True
                if prior is not None and np.max(np.abs(actual - prior)) <= STABLE_STEP_BAND:
                    stability_count += 1
                else:
                    stability_count = 0
                # Strict actual-position check: the Dynamixel In-Position
                # flag alone cannot establish arrival.
                if np.all(error <= 2) and stability_count >= STABILITY_SAMPLES and progress_seen:
                    break
                previous_abs_error = error
                prior = actual.copy()
                time.sleep(0.08)
            else:
                raise RuntimeError(
                    f'Waypoint {k} timed out before stable physical arrival; '
                    f'goal={waypoint.tolist()}, actual={actual.tolist()}, '
                    'no further commands sent.')
            previous_goal = waypoint.copy()
        self.get_logger().info(f'Movement finished. Actual={self.read_positions().tolist()}')

    def on_target(self, msg):
        if self.busy or self.completed:
            self.get_logger().warning('Request ignored; busy or already executed')
            return
        self.busy = True
        try:
            if len(msg.data) != 3:
                raise ValueError('Expected exactly [x, y, z] in meters')
            target = np.asarray(msg.data, dtype=float)
            if not np.all(np.isfinite(target)):
                raise ValueError('Target has NaN/Inf')
            current, goal, q0, q1, xyz, error, path = self.plan(target)
            print('\n====== SAFE XYZ PLAN ======', flush=True)
            print('Target XYZ [m]:', target.round(5))
            print('Current TCP [m]:', np.asarray(self.robot.fkine(q0).t).round(5))
            print('Calculated TCP [m]:', xyz.round(5))
            print('Current joint angles [deg]:', np.rad2deg(q0).round(2))
            print('Target joint angles [deg]:', np.rad2deg(q1).round(2))
            print('Motor start:', current.tolist())
            print('Motor goal: ', goal.tolist())
            print('Signed delta:', (goal-current).tolist())
            print('Model FK error [mm]:', round(1000*error, 3))
            print('Waypoints:', len(path))
            print('First waypoint:', path[0].tolist())
            print('Last waypoint: ', path[-1].tolist())
            print('NOTE: no link collision or pen-to-table collision checking')
            print('===========================\n', flush=True)
            self.print_motor_status()
            if not self.execute:
                self.get_logger().info('DRY RUN: zero writes / zero torque changes')
                return
            self.preflight_motion(current, goal)
            self.completed = True  # one physical operation per node start
            self.move_waypoints(path)
        except Exception as exc:
            self.get_logger().error(str(exc))
            if self.execute:
                self.completed = True  # latch closed on failure
        finally:
            self.busy = False

    def close(self):
        self.port.closePort()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute', action='store_true',
                        help='Request single physical move; still blocked until calibrated')
    opts, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = None
    try:
        node = MoveToPointSafe(execute=opts.execute)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
