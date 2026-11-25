
import numpy as np

from adatools import config_generator as cg
from spatialmath import SE3


robot = cg.get_robot_config_1(
    link1=0.141560,
    link1_offset=0.0,
    link2=0.371320,
    link2_offset=0.0,
    link3=0.210800,
    link3_offset=0.0,
    link4=0.222462,
    link4_offset=0.0
)

# Ønskt TCP-posisjon i robotbasens koordinatsystem
target = SE3(0.40, 0.10, 0.20)

# Startstilling for matematisk IK-test
q0 = np.zeros(robot.n)

solution = robot.ikine_LM(
    target,
    q0=q0,
    mask=[1, 1, 1, 0, 0, 0],
    joint_limits=True
)

if not solution.success:
    raise RuntimeError("IK failed")

q_target = solution.q

print("Leddvinklar i radianar:")
print(q_target)

print("\nLeddvinklar i grader:")
print(np.rad2deg(q_target))

print("\nKontroll av posisjon:")
print(robot.fkine(q_target).t)

print("\nNeste steg:")
print("Kalibrer motor-nullpunkt, retning og skala.")
print("Ingen kommandoar er sende til motorane.")
