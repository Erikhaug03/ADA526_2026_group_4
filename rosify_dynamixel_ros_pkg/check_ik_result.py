
import numpy as np
from spatialmath import SE3

from rosify_dynamixel_ros_pkg.robot_model import create_robot
from rosify_dynamixel_ros_pkg.motor_config import motor_to_joint

robot = create_robot()
robot.tool = SE3.Tz(0.080)

# Positions before physical IK test
motor_start = np.array([
    3107, 2274, 3176, 2980, 3146
])

# Last measured positions after IK test
motor_actual = np.array([
    3107, 2284, 3177, 2974, 3146
])

# Desired Cartesian target
target = np.array([
    0.20761, 0.01401, 0.19372
])

q_start = motor_to_joint(motor_start)
q_actual = motor_to_joint(motor_actual)

tcp_start = np.asarray(robot.fkine(q_start).t)
tcp_actual = np.asarray(robot.fkine(q_actual).t)

movement_mm = (tcp_actual - tcp_start) * 1000
error_mm = (target - tcp_actual) * 1000

print("\n========== IK RESULT ==========")

print("Start TCP [mm]:")
print(np.round(tcp_start * 1000, 2))

print("\nActual TCP from motor feedback [mm]:")
print(np.round(tcp_actual * 1000, 2))

print("\nEstimated TCP movement [mm]:")
print(np.round(movement_mm, 2))

print("\nTCP target error [mm]:")
print(np.round(error_mm, 2))

print("\nTotal position error [mm]:")
print(round(np.linalg.norm(error_mm), 2))

print("\nREAD ONLY - No motor commands sent")
