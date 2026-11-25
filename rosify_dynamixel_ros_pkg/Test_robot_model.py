import numpy as np
from robot_model import create_robot

robot = create_robot()

print("Robot type:", type(robot))
print("Robot model:", robot)

if hasattr(robot, "n"):
    print("Number of joints:", robot.n)

if hasattr(robot, "fkine") and hasattr(robot, "n"):

    q = np.zeros(robot.n)

    T = robot.fkine(q)

    print("\nForward kinematics:")
    print(T)

    print("\nTCP position:")
    print(T.t)

else:
    print("Robotmodellen har ikkje fkine() eller n.")