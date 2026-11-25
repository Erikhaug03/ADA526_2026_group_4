
import dynamixel_sdk as dxl

# Same kommunikasjonsinnstillingar som CustomDXL
PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

MOTOR_IDS = [1, 2, 3, 4, 5]

# Registeradresser frå CustomDXL.py
ADDR_PRESENT_POSITION = 132
ADDR_TORQUE_ENABLE = 64

port = dxl.PortHandler(PORT)
packet = dxl.PacketHandler(PROTOCOL_VERSION)

if not port.openPort():
    raise RuntimeError("Could not open Dynamixel port")

try:
    if not port.setBaudRate(BAUDRATE):
        raise RuntimeError("Could not set baudrate")

    print("\nDynamixel motor positions")
    print("----------------------------")

    for motor_id in MOTOR_IDS:

        # Les posisjon
        position, result, error = packet.read4ByteTxRx(
            port,
            motor_id,
            ADDR_PRESENT_POSITION
        )

        if result != dxl.COMM_SUCCESS or error != 0:
            print(f"Motor {motor_id}: READ ERROR")
            continue

        # Les torque status utan å endre han
        torque, result_t, error_t = packet.read1ByteTxRx(
            port,
            motor_id,
            ADDR_TORQUE_ENABLE
        )

        torque_status = (
            torque if result_t == dxl.COMM_SUCCESS
            and error_t == 0 else "unknown"
        )

        print(
            f"Motor {motor_id}: "
            f"position={position}, "
            f"torque={torque_status}"
        )

finally:
    port.closePort()

print("----------------------------")
print("Finished - no movement commands sent")
