
import numpy as np
from spatialmath import SE3

from robot_model import create_robot
from motor_config import Q_HOME

robot = create_robot()

print("Without TCP offset:")
print(robot.fkine(Q_HOME).t)

robot.tool = SE3.Tz(-0.080)

print("\nWith 80 mm pen offset:")
print(robot.fkine(Q_HOME).t)
