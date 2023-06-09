#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import serial
import socket
import threading

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 8082

class HttpComms(Node):

    def __init__(self):
        super().__init__('Arduino_http_comms')
        self.get_logger().info('Salut lume')

        self.serial_connection = serial.Serial('/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0', baudrate=115200)

        self.subscription = self.create_subscription(
            String,
            'arduino_raw_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ros_message = None
        self.running = True
        self.socket_thread = threading.Thread(target=self.socket_thread_function)
        self.socket_thread.daemon = True
        self.socket_thread.start()

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        self.ros_message = str.encode(msg.data)

    def socket_thread_function(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT))
            s.listen()
            conn, addr = s.accept()
            with conn:
                self.get_logger().info(f"Connected by {addr}")
                while self.running:
                    data = conn.recv(1024)
                    if not data:
                        break
                    data = data.replace(b'\n', b'\r')
                    self.serial_connection.write(data)
                    # self.get_logger().info(f"Sent: {data}")
                    arduino_data = self._readline()
                    conn.send(arduino_data)
                    # self.get_logger().info(f"Received: {arduino_data}")
                    if self.ros_message:
                        self.serial_connection.write(self.ros_message)
                        self.ros_message = None

    def _readline(self):
        eol = b'\r'
        leneol = len(eol)
        line = bytearray()
        while True:
            c = self.serial_connection.read(1)
            if c:
                line += c
                if line[-leneol:] == eol:
                    break
            else:
                break
        return bytes(line)



def main(args=None):
    rclpy.init(args=args)
    http_comms = HttpComms()
    
    while rclpy.ok():
        rclpy.spin_once(http_comms)
    running = False
    http_comms.serial_connection.close()
    http_comms.destroy_node()

    
if __name__ == '__main__':
    main()
