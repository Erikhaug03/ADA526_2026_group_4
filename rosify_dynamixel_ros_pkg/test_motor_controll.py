
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import numpy as np


# Må samsvare med namna frå /joint_states
JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5"
]

CONTROLLER_TOPIC = (
    "/joint_trajectory_controller/joint_trajectory"
)

# Første test: ingen faktisk motorkommando
DRY_RUN = True

# Test berre ei lita endring i joint 1
TEST_CHANGE = np.deg2rad(2)


class MotorControl(Node):

    def __init__(self):
        super().__init__("motor_control_test")

        self.current_q = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

        self.publisher = self.create_publisher(
            JointTrajectory,
            CONTROLLER_TOPIC,
            10
        )

        self.get_logger().info(
            "Motor control test started"
        )

    def joint_callback(self, msg):
        try:
            q = [
                msg.position[msg.name.index(name)]
                for name in JOINT_NAMES
            ]

            self.current_q = np.array(q, dtype=float)

        except (ValueError, IndexError):
            self.get_logger().warning(
                "Joint names stemmer ikkje med /joint_states"
            )

    def move_joints(self, q_target, duration=5.0):

        if self.current_q is None:
            self.get_logger().error(
                "Ingen joint states mottekne"
            )
            return

        q_target = np.asarray(q_target, dtype=float)

        if not np.all(np.isfinite(q_target)):
            self.get_logger().error("Ugyldige vinklar")
            return

        print("Current angles (degrees):")
        print(np.rad2deg(self.current_q))

        print("Target angles (degrees):")
        print(np.rad2deg(q_target))

        if DRY_RUN:
            self.get_logger().info(
                "DRY_RUN: Ingen kommando sendt"
            )
            return

        if self.publisher.get_subscription_count() == 0:
            self.get_logger().error(
                "Ingen controller lyttar på topicet"
            )
            return

        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = q_target.tolist()
        point.velocities = [0.0] * len(JOINT_NAMES)

        seconds = int(duration)
        nanoseconds = int((duration - seconds) * 1e9)

        point.time_from_start = Duration(
            sec=seconds,
            nanosec=nanoseconds
        )

        trajectory.points.append(point)
        self.publisher.publish(trajectory)

        self.get_logger().info("Trajectory published")


def main():

    rclpy.init()

    node = MotorControl()

    try:
        # Vent på at roboten sender leddstilling
        while rclpy.ok() and node.current_q is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        q_target = node.current_q.copy()
        q_target[0] += TEST_CHANGE

        node.move_joints(q_target, duration=5.0)

        # Gi ROS2 tid til å levere meldinga
        rclpy.spin_once(node, timeout_sec=0.5)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
