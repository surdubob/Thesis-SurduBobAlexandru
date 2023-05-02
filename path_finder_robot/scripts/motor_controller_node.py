#!/usr/bin/python3

import rclpy
from rclpy.node import Node

import pyrealsense2 as rs
import numpy as np
import cv2 as cv

class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('motor_commands_reader')


    

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ObjectDetectionNode()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()