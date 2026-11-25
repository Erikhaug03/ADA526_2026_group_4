import numpy as np

MOTOR_IDS = [1, 2, 3, 4, 5]

PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000

STEPS_PER_REVOLUTION = 4096
STEPS_PER_RADIAN = STEPS_PER_REVOLUTION / (2 * np.pi)

# Eksakte målte motorposisjonar i fysisk home
MOTOR_HOME = np.array([
    3062,
    2226,
    3153,
    2970,
    3146
])



# Positiv DH-rotasjon samanlikna med motorverdi
DIRECTIONS = np.array([
     1,
    -1,
    -1,
    -1,
     1
])

# FØREBELS DH-vinklar for fysisk home.
# Må verifiserast før fysisk IK-styring.
Q_HOME = np.deg2rad([
     0.0,
    90.0,
   -90.0,
     0.0,
     0.0
])

MOTOR_LIMITS = {
    1: (2245, 3775),
    2: (2194, 3275),
    3: (2139, 3640),
    4: (820, 3123),
    5: (0, 4095)
}


def motor_to_joint(motor_positions):
    motor_positions = np.asarray(
        motor_positions, dtype=float
    )

    if motor_positions.shape != (5,):
        raise ValueError("Expected five motor positions")

    return (
        Q_HOME
        + DIRECTIONS
        * (motor_positions - MOTOR_HOME)
        / STEPS_PER_RADIAN
    )


def joint_to_motor(q):
    q = np.asarray(q, dtype=float)

    if q.shape != (5,):
        raise ValueError("Expected five joint angles")

    if not np.all(np.isfinite(q)):
        raise ValueError("Invalid joint angles")

    positions = np.rint(
        MOTOR_HOME
        + DIRECTIONS
        * (q - Q_HOME)
        * STEPS_PER_RADIAN
    ).astype(int)

    for motor_id, position in zip(MOTOR_IDS, positions):
        minimum, maximum = MOTOR_LIMITS[motor_id]

        if not minimum <= position <= maximum:
            raise ValueError(
                f"Motor {motor_id}: {position} "
                f"outside [{minimum}, {maximum}]"
            )

    return positions.tolist()
