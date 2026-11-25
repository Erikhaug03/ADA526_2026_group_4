
import dynamixel_sdk as dxl

PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000

ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_MOVING = 122
ADDR_HARDWARE_ERROR = 70

port = dxl.PortHandler(PORT)
packet = dxl.PacketHandler(2.0)

def check(result, error):
    if result != dxl.COMM_SUCCESS:
        raise RuntimeError(packet.getTxRxResult(result))
    if error:
        raise RuntimeError(packet.getRxPacketError(error))

def signed32(value):
    return value - (1 << 32) if value >= (1 << 31) else value

if not port.openPort():
    raise RuntimeError("Could not open port")

try:
    if not port.setBaudRate(BAUDRATE):
        raise RuntimeError("Could not set baudrate")

    print("ID | Goal | Present | Error | Moving | HW Error")
    print("-" * 53)

    for motor_id in range(1, 6):
        goal, r, e = packet.read4ByteTxRx(
            port, motor_id, ADDR_GOAL_POSITION
        )
        check(r, e)

        present, r, e = packet.read4ByteTxRx(
            port, motor_id, ADDR_PRESENT_POSITION
        )
        check(r, e)

        moving, r, e = packet.read1ByteTxRx(
            port, motor_id, ADDR_MOVING
        )
        check(r, e)

        hardware, r, e = packet.read1ByteTxRx(
            port, motor_id, ADDR_HARDWARE_ERROR
        )
        check(r, e)

        goal = signed32(goal)
        present = signed32(present)

        print(
            f"{motor_id:2} | "
            f"{goal:5} | "
            f"{present:7} | "
            f"{present-goal:+5} | "
            f"{moving:6} | "
            f"{hardware:8}"
        )

finally:
    port.closePort()

print("\nREAD ONLY: No movement commands sent.")
