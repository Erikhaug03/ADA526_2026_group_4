import numpy as np

from motor_config import (
    MOTOR_HOME,
    Q_HOME,
    motor_to_joint,
    joint_to_motor
)

q = motor_to_joint(MOTOR_HOME)

print("DH angles at home:")
print(np.rad2deg(q))

positions = joint_to_motor(q)

print("\nMotor positions:")
print(positions)

print("\nCalibration consistency:")
print(np.allclose(positions, MOTOR_HOME))