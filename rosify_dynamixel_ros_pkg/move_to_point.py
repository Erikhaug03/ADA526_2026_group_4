
#!/usr/bin/env python3

import numpy as np
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from spatialmath import SE3

from custom_dxl.CustomDXL import CustomDXL

from rosify_dynamixel_ros_pkg.robot_model import create_robot
from rosify_dynamixel_ros_pkg.motor_config import (
    MOTOR_IDS,
    MOTOR_LIMITS,
    motor_to_joint,
    joint_to_motor
)


# =====================================
# CONFIGURATION
# =====================================

# No movement by default
MOTION_ENABLED = False

# Set only after physical calibration
# and safe movement tests are completed
CALIBRATION_VERIFIED = False

# Explicit acknowledgement for physical motion
MOTION_CONFIRMATION = ""

# Maximum movement per command (encoder steps)
MAX_STEP_CHANGE = 10

# Maximum allowed IK position error (meters)
POSITION_TOLERANCE = 0.005

# Current position may be slightly noisy
POSITION_STABILITY_TOLERANCE = 3


class MoveToPoint(Node):

    def __init__(self):

        super().__init__("move_to_point")

        self.robot = create_robot()

        # TCP-offset: pennespissen er 80 mm frå enderamma
        self.robot.tool = SE3.Tz(0.080)

        # Kontroller TCP
        q_home = np.deg2rad([0, 90, -90, 0, 0])

        self.get_logger().info(
            f"Robot tool:\n{self.robot.tool}"
        )

        self.get_logger().info(
            f"TCP at Q_HOME: {self.robot.fkine(q_home).t}"
        )

        self.dxls = CustomDXL(
            dxl_ids=MOTOR_IDS,
            port="/dev/ttyUSB0",
            baudrate=1000000
        )

        # Requires the modified open_port()
        # that does NOT enable torque automatically
        self.dxls.open_port()

        self.subscription = self.create_subscription(
            Float64MultiArray,
            "/target_point",
            self.target_callback,
            10
        )

        self.get_logger().info(
            "MoveToPoint node started"
        )

        self.get_logger().info(
            f"Motion enabled: {MOTION_ENABLED}"
        )

    # =====================================
    # READ MOTOR POSITIONS
    # =====================================

    def read_positions(self):

        positions = self.dxls.read_motor_positions()

        if positions is None or len(positions) != 5:
            raise RuntimeError(
                "Could not read all motor positions"
            )

        # Convert unsigned 32-bit to signed values
        positions = np.array([
            int(np.int32(p))
            for p in positions
        ], dtype=int)

        return positions

    # =====================================
    # CHECK TORQUE STATUS
    # =====================================

    def check_torque(self):

        import dynamixel_sdk as dxl

        for motor_id in MOTOR_IDS:

            value, result, error = (
                self.dxls.packetHandler.read1ByteTxRx(
                    self.dxls.portHandler,
                    motor_id,
                    64
                )
            )

            if result != dxl.COMM_SUCCESS or error != 0:
                raise RuntimeError(
                    f"Could not read torque for motor {motor_id}"
                )

            if value != 1:
                raise RuntimeError(
                    f"Motor {motor_id} torque is OFF"
                )

    # =====================================
    # TARGET POINT CALLBACK
    # =====================================

    def target_callback(self, msg):

        try:

            if len(msg.data) != 3:
                raise ValueError(
                    "Expected [x, y, z]"
                )

            target = np.array(
                msg.data, dtype=float
            )

            if not np.all(np.isfinite(target)):
                raise ValueError(
                    "Invalid target coordinates"
                )

            # Read actual motor positions
            motor_current = self.read_positions()

            # Convert motor positions to DH angles
            q_current = motor_to_joint(
                motor_current
            )

            # Calculate current TCP position
            current_xyz = np.asarray(
                self.robot.fkine(q_current).t
            )

            # =================================
            # INVERSE KINEMATICS
            # =================================

            T_target = SE3(*target)

            solution = self.robot.ikine_LM(
                T_target,
                q0=q_current,
                mask=[1, 1, 1, 0, 0, 0],
                joint_limits=True
            )

            if not solution.success:
                raise RuntimeError(
                    f"IK failed: {solution.reason}"
                )

            q_target = np.array(
                solution.q, dtype=float
            )

            # Keep joint 5 at its current angle
            # during the first positioning tests
            q_target[4] = q_current[4]

            # Verify forward kinematics
            calculated_xyz = np.asarray(
                self.robot.fkine(q_target).t
            )

            error = np.linalg.norm(
                calculated_xyz - target
            )

            if error > POSITION_TOLERANCE:
                raise RuntimeError(
                    f"IK position error: {error:.5f} m"
                )

            # Convert target angles to motors
            motor_target = np.array(
                joint_to_motor(q_target),
                dtype=int
            )

            # Hold current raw position on motor 5
            motor_target[4] = motor_current[4]

            # Check limits
            for motor_id, position in zip(
                MOTOR_IDS, motor_target
            ):
                minimum, maximum = MOTOR_LIMITS[motor_id]

                if not minimum <= position <= maximum:
                    raise RuntimeError(
                        f"Motor {motor_id} outside limits"
                    )

            delta = np.abs(
                motor_target - motor_current
            )

            # =================================
            # PRINT RESULTS
            # =================================

            print("\n========== MOVE TO POINT ==========")

            print("Target XYZ:", target.round(4))
            print("Current XYZ:", current_xyz.round(4))

            print("\nCurrent joint angles:")
            print(np.rad2deg(q_current).round(2))

            print("\nTarget joint angles:")
            print(np.rad2deg(q_target).round(2))

            print("\nCurrent motor positions:")
            print(motor_current.tolist())

            print("\nTarget motor positions:")
            print(motor_target.tolist())

            print("\nMotor movement (steps):")
            print(delta.tolist())

            print("\nPredicted FK error:")
            print(f"{error * 1000:.2f} mm")

            print("===================================\n")

            # =================================
            # SAFETY CHECKS
            # =================================

            if not MOTION_ENABLED:
                self.get_logger().info(
                    "DRY RUN: No motor commands sent"
                )
                return

            if not CALIBRATION_VERIFIED:
                raise RuntimeError(
                    "Physical calibration not verified"
                )

            if MOTION_CONFIRMATION != "CALIBRATED_AND_CHECKED":
                raise RuntimeError(
                    "Motion confirmation missing"
                )

            if np.any(delta > MAX_STEP_CHANGE):
                raise RuntimeError(
                    "Movement too large for first test"
                )

            # No unexpected motor movement during calculation
            motor_check = self.read_positions()

            if np.any(
                np.abs(motor_check - motor_current)
                > POSITION_STABILITY_TOLERANCE
            ):
                raise RuntimeError(
                    "Motor position changed during planning"
                )

            # Do not enable torque automatically
            self.check_torque()

            # =================================
            # PHYSICAL MOVEMENT
            # =================================

            self.dxls.send_goal(
                goal_pos=motor_target.tolist()
            )

            self.get_logger().info(
                "Small motor command sent"
            )

            # Read back immediately
            # This is NOT confirmation of arrival
            actual = self.read_positions()

            print("Readback positions:")
            print(actual.tolist())

        except Exception as error:

            self.get_logger().error(
                str(error)
            )

    def close(self):
        self.dxls.portHandler.closePort()


def main(args=None):

    rclpy.init(args=args)

    node = MoveToPoint()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
