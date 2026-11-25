
import numpy as np
from spatialmath import SE3

from rosify_dynamixel_ros_pkg.robot_model import create_robot
from rosify_dynamixel_ros_pkg.motor_config import (
    MOTOR_IDS,
    MOTOR_LIMITS,
    motor_to_joint,
)

robot = create_robot()
robot.tool = SE3.Tz(0.080)

# Last measured motor positions
motor_current = np.array([
    3089, 2256, 3160, 2971, 3146
])

TEST_STEPS = 10

q_current = motor_to_joint(motor_current)
tcp_current = np.asarray(
    robot.fkine(q_current).t
).reshape(3)

print("Current TCP [mm]:")
print(np.round(tcp_current * 1000, 2))

print("\nExpected TCP movement for +10 motor steps:")

for idx, motor_id in enumerate(MOTOR_IDS):

    if motor_id == 5:
        continue

    motor_test = motor_current.copy()
    motor_test[idx] += TEST_STEPS

    lower, upper = MOTOR_LIMITS[motor_id]

    if not lower <= motor_test[idx] <= upper:
        print(f"Motor {motor_id}: outside limits")
        continue

    q_test = motor_to_joint(motor_test)

    tcp_test = np.asarray(
        robot.fkine(q_test).t
    ).reshape(3)

    delta_mm = (tcp_test - tcp_current) * 1000

    print(
        f"Motor {motor_id}: "
        f"dX={delta_mm[0]:+.2f} mm, "
        f"dY={delta_mm[1]:+.2f} mm, "
        f"dZ={delta_mm[2]:+.2f} mm"
    )

print("\nREAD ONLY: No motor commands sent.")
