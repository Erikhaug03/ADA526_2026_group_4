
import dynamixel_sdk as dxl
import time

# ------------------------------------
# CONFIGURATION
# ------------------------------------

PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000
PROTOCOL = 2.0

MOTOR_ID = 2

MIN_POSITION = 2194
MAX_POSITION = 3275

MOVEMENT = 10

PROFILE_VELOCITY = 5
PROFILE_ACCELERATION = 2

DRY_RUN = False

# Dynamixel register addresses
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132


port = dxl.PortHandler(PORT)
packet = dxl.PacketHandler(PROTOCOL)


def check_result(result, error):
    if result != dxl.COMM_SUCCESS:
        raise RuntimeError(packet.getTxRxResult(result))

    if error != 0:
        raise RuntimeError(packet.getRxPacketError(error))


def read_position():
    position, result, error = packet.read4ByteTxRx(
        port, MOTOR_ID, ADDR_PRESENT_POSITION
    )
    check_result(result, error)
    return position


def write_position(position):
    result, error = packet.write4ByteTxRx(
        port, MOTOR_ID,
        ADDR_GOAL_POSITION,
        int(position)
    )
    check_result(result, error)


def main():

    if not port.openPort():
        raise RuntimeError("Could not open port")

    try:
        if not port.setBaudRate(BAUDRATE):
            raise RuntimeError("Could not set baudrate")

        mode, result, error = packet.read1ByteTxRx(
            port, MOTOR_ID, ADDR_OPERATING_MODE
        )
        check_result(result, error)

        if mode != 3:
            raise RuntimeError(
                f"Expected position mode 3, got {mode}"
            )

        torque, result, error = packet.read1ByteTxRx(
            port, MOTOR_ID, ADDR_TORQUE_ENABLE
        )
        check_result(result, error)

        current = read_position()
        target = current + MOVEMENT

        print("Motor ID:", MOTOR_ID)
        print("Current position:", current)
        print("Target position:", target)
        print("Torque:", torque)

        if not MIN_POSITION <= current <= MAX_POSITION:
            raise RuntimeError(
                "Current position outside safe limits"
            )

        if not MIN_POSITION <= target <= MAX_POSITION:
            raise RuntimeError(
                "Target outside safe limits"
            )

        if DRY_RUN:
            print("DRY_RUN: No movement sent")
            return

        confirmation = input(
            "Type MOVE to activate and move motor 1: "
        )

        if confirmation != "MOVE":
            print("Cancelled")
            return

        # Configure low velocity and acceleration
        result, error = packet.write4ByteTxRx(
            port, MOTOR_ID,
            ADDR_PROFILE_VELOCITY,
            PROFILE_VELOCITY
        )
        check_result(result, error)

        result, error = packet.write4ByteTxRx(
            port, MOTOR_ID,
            ADDR_PROFILE_ACCELERATION,
            PROFILE_ACCELERATION
        )
        check_result(result, error)

        # Set target equal to current position
        # before enabling torque
        if torque == 0:
            write_position(current)

            result, error = packet.write1ByteTxRx(
                port, MOTOR_ID, ADDR_TORQUE_ENABLE, 1
            )
            check_result(result, error)

            time.sleep(0.3)

            # Verify position after torque enable
            actual = read_position()

            if abs(actual - current) > 5:
                raise RuntimeError(
                    "Position changed after torque enable"
                )

        # Small test movement
        print("Moving motor...")
        write_position(target)

        time.sleep(2.0)

        actual = read_position()

        print("Final position:", actual)
        print("Position error:", target - actual)
        print("Test finished")

    finally:
        port.closePort()


if __name__ == "__main__":
    main()
