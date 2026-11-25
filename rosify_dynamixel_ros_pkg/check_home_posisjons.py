
import numpy as np

from robot_model import create_robot
from motor_config import Q_HOME

robot = create_robot()

T_home = robot.fkine(Q_HOME)

print("DH home angles (degrees):")
print(np.rad2deg(Q_HOME))

print("\nCalculated TCP position (meters):")
print(T_home.t)

print("\nCalculated orientation (degrees):")
print(np.rad2deg(T_home.rpy(order="xyz")))
