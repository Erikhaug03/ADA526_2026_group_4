#!/usr/bin/env python3
"""One-shot, bounded 20 mm Cartesian IK motion trial for the ADA526 arm.

Dry-run by default. No automatic torque activation. This is NOT a
collision checker or an emergency stop. Operator must check surroundings.
"""
import argparse
import time
import numpy as np
import dynamixel_sdk as dxl
from spatialmath import SE3
from rosify_dynamixel_ros_pkg.robot_model import create_robot
from rosify_dynamixel_ros_pkg.motor_config import (
    MOTOR_IDS, MOTOR_LIMITS, motor_to_joint, joint_to_motor,
)

PORT = '/dev/ttyUSB0'
BAUD = 1000000
ADDR_MODE = 11
ADDR_TORQUE = 64
ADDR_HW_ERROR = 70
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL = 116
ADDR_PRESENT = 132
MAX_MOTOR_DELTA = 25  # Counts measured from current position, not old goal
MAX_TCP_DELTA_M = 0.020  # 20 mm or less
PROFILE_VELOCITY = 5
PROFILE_ACCELERATION = 2


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute', action='store_true', help='Enable ONE small physical movement')
    ap.add_argument('--dx-mm', type=float, default=50.0)
    ap.add_argument('--dy-mm', type=float, default=0.0)
    ap.add_argument('--dz-mm', type=float, default=0.0)
    args = ap.parse_args()

    delta_m = np.array([args.dx_mm, args.dy_mm, args.dz_mm], dtype=float) / 1000
    if not np.all(np.isfinite(delta_m)) or not 0 < np.linalg.norm(delta_m) <= MAX_TCP_DELTA_M + 1e-12:
        raise ValueError('Requested displacement must be >0 and <=20 mm')

    port = dxl.PortHandler(PORT)
    packet = dxl.PacketHandler(2.0)

    def check(result, error, label):
        if result != dxl.COMM_SUCCESS or error:
            raise RuntimeError(f'{label}: {packet.getTxRxResult(result)} / {packet.getRxPacketError(error)}')

    def read1(i, addr):
        x, r, e = packet.read1ByteTxRx(port, i, addr)
        check(r, e, f'read 1B id={i} addr={addr}')
        return x

    def read4(i, addr):
        x, r, e = packet.read4ByteTxRx(port, i, addr)
        check(r, e, f'read 4B id={i} addr={addr}')
        return x if x < 2**31 else x - 2**32

    def write4(i, addr, val):
        r, e = packet.write4ByteTxRx(port, i, addr, int(val))
        check(r, e, f'write 4B id={i} addr={addr}')

    def read_positions():
        return np.array([read4(i, ADDR_PRESENT) for i in MOTOR_IDS], dtype=int)

    def validate(p):
        for i, pos in zip(MOTOR_IDS, p):
            lo, hi = MOTOR_LIMITS[i]
            if not lo <= int(pos) <= hi:
                raise RuntimeError(f'Motor {i} at {pos}: outside allowed [{lo}, {hi}]')

    if not port.openPort():
        raise RuntimeError('Failed to open Dynamixel port')
    try:
        if not port.setBaudRate(BAUD):
            raise RuntimeError('Failed to set baudrate')

        robot = create_robot()
        robot.tool = SE3.Tz(0.080)
        start = read_positions()
        validate(start)
        q0 = motor_to_joint(start)
        xyz0 = np.asarray(robot.fkine(q0).t, dtype=float).reshape(3)
        target_xyz = xyz0 + delta_m
        ik = robot.ikine_LM(
            SE3(*target_xyz), q0=q0,
            mask=[1, 1, 1, 0, 0, 0], joint_limits=True)
        if not ik.success or ik.q is None:
            raise RuntimeError(f'IK failed: {ik.reason}')
        q1 = np.asarray(ik.q, dtype=float).copy()
        q1[4] = q0[4]
        goal = np.asarray(joint_to_motor(q1), dtype=int)
        goal[4] = start[4]
        validate(goal)
        motor_delta = goal - start
        if np.max(np.abs(motor_delta[:4])) > MAX_MOTOR_DELTA:
            raise RuntimeError(f'Motor displacement exceeds 25 steps: {motor_delta.tolist()}')
        predicted = np.asarray(robot.fkine(q1).t, dtype=float).reshape(3)
        if np.linalg.norm(predicted - target_xyz) > 0.001:
            raise RuntimeError('IK residual >1 mm')

        # Check representative interpolated poses: only TCP-point above model base,
        # not physical collision verification.
        samples = max(9, int(np.max(np.abs(motor_delta))) + 1)
        min_tcp_z = min(float(robot.fkine(motor_to_joint(
            np.rint(start + a * motor_delta).astype(int))).t[2]) for a in np.linspace(0, 1, samples))
        if min_tcp_z < 0.15:
            raise RuntimeError('Model TCP too low (<150 mm from uncalibrated DH origin)')

        print('\n========= ONE-SHOT IK TRIAL =========')
        print('Mode:', 'PHYSICAL TEST' if args.execute else 'READ-ONLY DRY RUN')
        print('Measured motor start:', start.tolist())
        print('Measured TCP (model) [mm]:', np.round(xyz0 * 1000, 2))
        print('Requested delta XYZ [mm]:', np.round(delta_m * 1000, 2))
        print('Goal TCP (model) [mm]:', np.round(target_xyz * 1000, 2))
        print('Motor goal:', goal.tolist())
        print('Motor deltas:', motor_delta.tolist())
        print('Min interpolated TCP base Z [mm]:', round(min_tcp_z * 1000, 2))
        print('No collision model; verify ALL link, pen and cable clearances manually.')
        print('The robot may sag or respond unexpectedly. Ensure adequate full-arm clearance.')
        print('Do NOT execute if the predicted goal or motion is different from expectation.')
        print('=====================================\n')
        if not args.execute:
            print('DRY RUN: no writes, no torque changes, no motion.')
            return

        for i in MOTOR_IDS:
            expected_mode = 5 if i == 5 else 3
            if read1(i, ADDR_MODE) != expected_mode:
                raise RuntimeError(f'Motor {i} is not in required mode {expected_mode}')
            if read1(i, ADDR_HW_ERROR):
                raise RuntimeError(f'Motor {i} has hardware error')
            if read1(i, ADDR_TORQUE) != 1:
                raise RuntimeError(f'Motor {i} torque OFF; no automatic enabling')

        print('WARNING: Program stop / Ctrl+C is NOT an emergency stop.')
        print('Ensure clear path, secured robot, supervised test, accessible hardware E-stop.')
        phrase = input('Type MOVE ONCE to send a single IK goal, else cancel: ')
        if phrase != 'MOVE ONCE':
            print('Cancelled; zero writes.')
            return

        fresh = read_positions()
        if np.max(np.abs(fresh - start)) > 2:
            raise RuntimeError(f'Position changed since plan: {fresh.tolist()}')
        validate(fresh)
        # Protect against motion while profile registers are updated.
        for i in MOTOR_IDS[:4]:
            write4(i, ADDR_PROFILE_ACCEL, PROFILE_ACCELERATION)
            write4(i, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY)
        # No chain of tiny waypoints: only one bounded goal per motor.
        # Sequential writes are not synchronized. Monitor and report only.
        for i, p in zip(MOTOR_IDS[:4], goal[:4]):
            write4(i, ADDR_GOAL, p)
        print('Goal commands sent to motors 1-4 ONCE. Motor 5 unchanged.')
        # Read actual positions at intervals, WITHOUT sending more goals.
        for k in range(10):
            time.sleep(0.2)
            actual = read_positions()
            print(f'Feedback {(k+1)*0.2:.1f}s: {actual.tolist()}, '
                  f'error: {(actual-goal).tolist()}', flush=True)
            for i in MOTOR_IDS:
                if read1(i, ADDR_HW_ERROR):
                    print(f'WARNING: Hardware fault ID {i}. No further goals will be sent.')
                    return
        final_actual = read_positions()
        q_actual = motor_to_joint(final_actual)
        actual_xyz = np.asarray(robot.fkine(q_actual).t, dtype=float).reshape(3)
        print('\n========= FINAL FK FEEDBACK =========')
        print('Actual motor positions:', final_actual.tolist())
        print('Goal motor positions:  ', goal.tolist())
        print('Actual TCP estimated from encoder [mm]:', np.round(actual_xyz * 1000, 2))
        print('Estimated TCP displacement [mm]:', np.round((actual_xyz - xyz0) * 1000, 2))
        print('Target TCP error [mm]:', np.round((target_xyz - actual_xyz) * 1000, 2))
        print('3D model-based error [mm]:', round(float(np.linalg.norm(target_xyz - actual_xyz)) * 1000, 2))
        print('=====================================')
        print('Trial complete; compare observed direction/physical pen displacement.')
        print('A persistent goal error is NOT silently accepted as arrival.')
    finally:
        port.closePort()

if __name__ == '__main__':
    main()
