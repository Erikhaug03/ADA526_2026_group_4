import dynamixel_sdk as dxl

port = dxl.PortHandler("/dev/ttyUSB0")
packet = dxl.PacketHandler(2.0)

if not port.openPort():
    raise RuntimeError("Could not open port")

try:
    if not port.setBaudRate(1000000):
        raise RuntimeError("Could not set baudrate")

    for motor_id in range(1, 6):

        model, result, error = packet.ping(
            port, motor_id
        )

        if result != dxl.COMM_SUCCESS or error != 0:
            print(f"Motor {motor_id}: Ping failed")
            continue

        mode, result, error = packet.read1ByteTxRx(
            port, motor_id, 11
        )

        if result == dxl.COMM_SUCCESS and error == 0:
            print(
                f"Motor {motor_id}: "
                f"Model={model}, Operating mode={mode}"
            )
        else:
            print(f"Motor {motor_id}: Mode read failed")

finally:
    port.closePort()