#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from smbus2 import SMBus

bus = SMBus(1)
address = 0x21

class MotorController(Node):

    def __init__(self):
        super().__init__('motor_commands_reader')
        self.subscription = self.create_subscription(String, '/web_movement_commands', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
    
    def StringToBytes(self, val):
        retVal = []
        for c in val:
                retVal.append(ord(c))
        return retVal

    def listener_callback(self, msg):
        byteValue = self.StringToBytes(msg.data)   
        try:
            bus.write_i2c_block_data(address, 0x00, byteValue)
        except:
            pass


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MotorController()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()