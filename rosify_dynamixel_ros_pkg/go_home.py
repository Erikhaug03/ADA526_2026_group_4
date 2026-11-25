
#!/usr/bin/env python3

import time
from dynamixel_sdk import PortHandler, PacketHandler

PORT = "/dev/ttyUSB0"
BAUD = 1000000

MOTOR_HOME = [3062, 2226, 3143, 2970, 3146]

ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116

port = PortHandler(PORT)
packet = PacketHandler(2.0)

if not port.openPort():
    raise RuntimeError("Could not open port")

port.setBaudRate(BAUD)

try:
    print("Moving robot to HOME...")

    for motor_id, home in enumerate(MOTOR_HOME, start=1):

        # Rolig bevegelse
        packet.write4ByteTxRx(
            port, motor_id,
            ADDR_PROFILE_ACCELERATION, 500
        )

        packet.write4ByteTxRx(
            port, motor_id,
            ADDR_PROFILE_VELOCITY, 3000
        )

        packet.write4ByteTxRx(
            port, motor_id,
            ADDR_GOAL_POSITION, home
        )

    time.sleep(5)
    print("HOME command sent.")

finally:
    port.closePort()
