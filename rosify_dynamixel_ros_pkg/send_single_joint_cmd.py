#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from custom_dxl.CustomDXL import CustomDXL

from std_msgs.msg import Int32

class myDynamixelController(Node):
    def __init__(self) -> None:
        super().__init__("my_dynamixel_controller")
        self.sub = self.create_subscription(Int32, '/dxl_joint_cmd', self.listener_callback, 10)
        self.dxls = CustomDXL(dxl_ids=[3]) ## TODO set your own dynamixel motor IDs
        self.dxls.open_port()
        self.dxls.send_goal(goal_pos=[1000]) ## TODO Adjust position commands according to your motors, give a list of all joints.
        print("Created")

    def listener_callback(self, msg):
        motor_cmd = msg.data 
        self.dxls.send_single_goal(motor_order = 0, goal_pos = motor_cmd)
    
def main(args=None):
    rclpy.init(args=args)
    node = myDynamixelController()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()