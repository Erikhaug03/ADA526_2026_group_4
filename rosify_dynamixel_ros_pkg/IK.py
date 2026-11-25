
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3

from rosify_dynamixel_ros_pkg.robot_model import create_robot
from rosify_dynamixel_ros_pkg.motor_config import joint_to_motor

# Opprett robotmodellen
robot = create_robot()
robot.tool = SE3.Tz(0.080)

# Startvinklar [rad]
q = np.array([0, np.pi/2, -np.pi/2, 0, 0])

# Punkt [X, Y, Z] i meter, relativt til DH-origo
points = [
    [0.275, -0.125, 0.006]
]

# Rekn ut IK for kvart punkt
for i, xyz in enumerate(points, start=1):

    target = SE3(*xyz)

    solution = robot.ikine_LM(
        target,
        q0=q,
        mask=[1, 1, 1, 0, 0, 0],
        joint_limits=True
    )

    if not solution.success:
        print(f"Punkt {i}: Ingen IK-løysing")
        continue

    q = np.array(solution.q)
    q[4] = 0.0  # Held motor 5 i heimvinkelen

    print(f"\n========== Punkt {i} ==========")
    print("Mål XYZ [m]:", xyz)
    print("Vinklar [rad]:", np.round(q, 4))
    print("Vinklar [grader]:", np.round(np.rad2deg(q), 2))
    print("Motorsteg:", joint_to_motor(q))
    print("FK XYZ [m]:", np.round(robot.fkine(q).t, 4))

    # Plot robotarmen i 3D
    robot.plot(
        q,
        backend="pyplot",
        block=True,
        jointaxes=True,
        eeframe=True
    )

    plt.title(f"Robotarm – Punkt {i}: {xyz}")

# Vis alle figurane
plt.show()

