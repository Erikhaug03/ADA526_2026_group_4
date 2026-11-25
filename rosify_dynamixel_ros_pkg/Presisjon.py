
import numpy as np
from spatialmath import SE3
from roboticstoolbox import jtraj, ctraj

from rosify_dynamixel_ros_pkg.robot_model import create_robot
from rosify_dynamixel_ros_pkg.motor_config import motor_to_joint

robot = create_robot()
robot.tool = SE3.Tz(0.080)

# Eksempel: erstatt med ferske motorposisjonar
motor_pos = np.array([3105, 2284, 3177, 2973, 3146])
q_start = motor_to_joint(motor_pos)

# Målet, i meter relativt til DH-origo
target_xyz = np.array([0.275, 0.12, 0.01])

# 50 mm over målet
above_xyz = target_xyz + [0, 0, 0.05]

# Ønskt verktøyretning
R_down = SE3.Rx(np.pi)
T_above = SE3.Trans(*above_xyz) * R_down
T_target = SE3.Trans(*target_xyz) * R_down

mask = [1, 1, 1, 1, 1, 0]

def solve_ik(T, q_seed):
    sol = robot.ikine_LM(
        T,
        q0=q_seed,
        mask=mask,
        joint_limits=True
    )
    if not sol.success:
        raise RuntimeError("IK failed")

    q = np.asarray(sol.q, dtype=float)
    T_actual = robot.fkine(q)

    pos_error = np.linalg.norm(T_actual.t - T.t)
    tool_z = T_actual.R[:, 2]
    angle_error = np.arccos(
        np.clip(np.dot(tool_z, [0, 0, -1]), -1, 1)
    )

    if pos_error > 0.002 or angle_error > np.deg2rad(2):
        raise RuntimeError("Position/orientation tolerance exceeded")

    return q

# FASE 1: Finn leddvinklar over målet
q_above = solve_ik(T_above, q_start)

# JTRAJ: 5 sekund, 50 punkt
t1 = np.linspace(0, 5, 50)
traj1 = jtraj(q_start, q_above, t1)

# FASE 2: Rett ned i kartesisk rom
# 5 sekund, 50 punkt, med jamn avstand
poses = ctraj(
    T_above,
    T_target,
    s=np.linspace(0, 1, 50)
)

q_previous = q_above.copy()
joint_path2 = []

for T in poses:
    q_previous = solve_ik(T, q_previous)
    joint_path2.append(q_previous.copy())

joint_path2 = np.array(joint_path2)

# Kontroller om jtraj held pennen nedover
max_tilt = 0.0
for q in traj1.q:
    tool_z = robot.fkine(q).R[:, 2]
    tilt = np.arccos(
        np.clip(np.dot(tool_z, [0, 0, -1]), -1, 1)
    )
    max_tilt = max(max_tilt, tilt)

print("Above target [m]:", above_xyz)
print("Final target [m]:", target_xyz)
print("Max tilt during jtraj [deg]:",
      np.rad2deg(max_tilt))

print("Joint path 1:", traj1.q.shape)
print("Joint path 2:", joint_path2.shape)
print("Planning complete. No motor commands sent.")
