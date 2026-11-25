
import dynamixel_sdk as dxl

PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000

port = dxl.PortHandler(PORT)
packet = dxl.PacketHandler(2.0)


def signed16(value):
    return value - 65536 if value >= 32768 else value


def signed32(value):
    return value - 4294967296 if value >= 2147483648 else value


def read(motor_id, address, size):
    if size == 1:
        value, result, error = packet.read1ByteTxRx(
            port, motor_id, address
        )
    elif size == 2:
        value, result, error = packet.read2ByteTxRx(
            port, motor_id, address
        )
    else:
        value, result, error = packet.read4ByteTxRx(
            port, motor_id, address
        )

    if result != dxl.COMM_SUCCESS:
        raise RuntimeError(packet.getTxRxResult(result))

    if error:
        raise RuntimeError(packet.getRxPacketError(error))

    return value


if not port.openPort():
    raise RuntimeError("Failed to open port")

try:
    if not port.setBaudRate(BAUDRATE):
        raise RuntimeError("Failed to set baudrate")

    for motor_id in [1, 2, 3, 4, 5]:

        goal = signed32(read(motor_id, 116, 4))
        position = signed32(read(motor_id, 132, 4))
        current = signed16(read(motor_id, 126, 2))
        pwm = signed16(read(motor_id, 124, 2))
        velocity = signed32(read(motor_id, 128, 4))
        moving_status = read(motor_id, 123, 1)
        p_gain = read(motor_id, 84, 2)

        print(f"\nMotor {motor_id}")
        print(f"  Goal Position:    {goal}")
        print(f"  Present Position: {position}")
        print(f"  Position Error:   {position - goal}")
        print(f"  Present Current:  {current} raw")
        print(f"  Present PWM:      {pwm} raw")
        print(f"  Present Velocity: {velocity} raw")
        print(f"  Moving Status:    {moving_status}")
        print(f"  Position P Gain:  {p_gain}")

finally:
    port.closePort()

print("\nREAD ONLY - no motor commands sent")
