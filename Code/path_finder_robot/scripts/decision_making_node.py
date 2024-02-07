#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist 

import time

import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
import torch
from scipy.interpolate import interp1d
from enum import Enum
import math

from threading import Timer, Thread

debug = False

class RobotState(Enum):
    NAVIGATION = 1
    APPROACHING = 2
    CLOSE = 3
    GRABBING = 4
    GRABBED = 5


class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('decision_making_node')
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self.camera_frame_received,
            10)
        
        self.decision_making_state = RobotState.APPROACHING

        self.camera_subscription
        self.camera_distance_calibration_x = [10, 23, 47, 81, 92, 132, 170, 200, 250, 270, 320, 370, 350] # pixels for realsense
        self.camera_distance_calibration_y = [350, 282, 140, 78, 66, 50, 38, 32, 27, 24, 16, 13, 11] # cm
        # self.camera_distance_calibration_x = [20, 36, 68, 78, 105, 140, 165, 190, 218, 315, 404, 440] # pixels for raspicam
        # self.camera_distance_calibration_y = [282, 140, 78, 66, 50, 38, 32, 27, 24, 16, 13, 11] # cm

        self.distance_interp_function = interp1d(self.camera_distance_calibration_x, self.camera_distance_calibration_y)

        self.br = CvBridge()
        self.running = True
        self.last_mushroom_dist = 0
        self.last_mushroom_center = (0, 0)
        self.last_bad_angle_time = 0
        self.detected_once = False
        self.last_results = None

        self.processed_publisher = self.create_publisher(CompressedImage,
                                                         '/mushroom_detection_image/compressed',
                                                         10)
        
        self.motor_command_publisher = self.create_publisher(String, '/arduino_raw_commands', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_decision_making', 10)
        
        time.sleep(0.5)
        self.release_mushroom()
        
        if debug:
            cv.namedWindow('Detections')
            cv.startWindowThread()

        self.model =  torch.hub.load('install/path_finder_robot/share/path_finder_robot/yolov5-deploy/yolov5', 'custom', source ='local', path='install/path_finder_robot/share/path_finder_robot/yolov5-deploy//mushroom.pt',force_reload=True) ### The repo is stored locally

        self.decision_making_thread = Thread(target=self.decision_making_function)
        self.decision_making_thread.start()

        self.classes = self.model.names ### class names in string format  
        self.start_time = time.time()

    def camera_frame_received(self, msg):
        image = self.br.compressed_imgmsg_to_cv2(msg)
        self.frame_width = image.shape[1]
        self.frame_height = image.shape[0]
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        results = self.detectx(image, model=self.model)
        image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
        image = self.plot_boxes_and_calculate_distance(results, image, classes=self.classes)
        
        cv.line(image, (int(self.frame_width / 2 + 70), 0), (int(self.frame_width / 2 + 70), self.frame_width), (255, 0, 0), 3)
        cv.line(image, (int(self.frame_width / 2 + 140), 0), (int(self.frame_width / 2 + 140), self.frame_width), (255, 0, 0), 3)

        self.last_results = results

        pbls = CompressedImage()
        pbls.format = "jpeg"
        pbls.data = cv.imencode('.jpg', image)[1].tobytes()
        
        self.processed_publisher.publish(pbls)

        if debug:
            cv.imshow('Detections', image)
            if cv.waitKey(1) & 0xFF == ord('q'):
                self.running = False


    def calculate_control_angular_speed(self):
        angle = math.atan((self.frame_width / 2 + 100 - self.last_mushroom_center[0]) / self.last_mushroom_dist)
        return angle

    def decision_making_function(self):
        while rclpy.ok():
            if self.last_results is not None:
                labels, cord = self.last_results
                if self.detected_once:
                    if self.decision_making_state == RobotState.APPROACHING:
                        if self.last_mushroom_dist > 40 and len(labels) > 0 and cord[0][4] > 0.7:
                            angle = self.calculate_control_angular_speed()
                            self.move_command(0.2, angle * 0.6, speed=0.03)
                        else:
                            self.move_command(0.0, 0.0, 0.0)
                            if len(labels) > 0 and cord[0][4] > 0.7:
                                self.decision_making_state = RobotState.CLOSE
                                self._logger.info('mushroom is close')

                    if self.decision_making_state == RobotState.CLOSE:
                        angle = 0 # self.calculate_control_angular_speed()
                        if self.last_mushroom_center[0] > self.frame_width / 2 + 140:
                            angle = -1
                            self.last_bad_angle_time = time.time()
                        elif self.last_mushroom_center[0] < self.frame_width / 2 + 70:
                            angle = 1
                            self.last_bad_angle_time = time.time()

                        if angle == 0 and time.time() - self.last_bad_angle_time > 1:
                            self._logger.info('direction is good')
                            self.decision_making_state = RobotState.GRABBING
                        elif angle != 0:
                            self._logger.info('angle still not ok, angle: ' + str(angle))
                            self.last_bad_angle_time = time.time()
                            self.move_command(0.0, angle * 2.0, speed=0.5)
                            time.sleep(0.6)
                            self.move_command(0.0, 0.0, speed=0.000)
                            time.sleep(0.6)

                    if self.decision_making_state == RobotState.GRABBING:
                        print(self.last_mushroom_dist)
                        if self.last_mushroom_dist > 11 and len(labels) > 0 and cord[0][4] > 0.7:
                            self.move_command(0.1, 0.0, speed=0.005)
                        elif len(labels) > 0 and cord[0][4] > 0.7:
                            self.grab_mushroom()
                            self.decision_making_state = RobotState.GRABBED
            time.sleep(0.33)


    def detectx (self, frame, model):
        frame = [frame]
        results = model(frame)
        # results.show()
        # print( results.xyxyn[0])
        # print(results.xyxyn[0][:, -1])
        # print(results.xyxyn[0][:, :-1])

        labels, cordinates = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]

        return labels, cordinates


    def plot_boxes_and_calculate_distance(self, results, frame, classes):
        labels, cord = results
        n = len(labels)
        x_shape, y_shape = frame.shape[1], frame.shape[0]

        ### looping through the detections
        for i in range(n):
            row = cord[i]
            if row[4] >= 0.7: ### threshold value for detection. We are discarding everything below this value
                x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape) ## BBOx coordniates
                text_d = classes[int(labels[i])]

                if text_d == 'mushroom':
                    cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2) ## BBox
                    cv.rectangle(frame, (x1, y1-20), (x2, y1), (0, 255,0), -1) ## for text label background

                    center = (int((x1 + x2) / 2), int((y1 + y2) / 2))

                    if abs(x1 - x2) < 60 and abs(y1 - y2) < 60 and center[1] > self.frame_height - 100:
                        continue

                    # print('x1: ', x1, '            x2: ', x2)
                    cv.circle(frame, (int((x1 + x2) / 2), int((y1 + y2) / 2)), 3, (255, 0, 0), -1)

                    apparent_width = x2 - x1

                    dist = 0
                    if 23 < apparent_width < 350:
                        dist = int(self.distance_interp_function(apparent_width))
                    elif apparent_width <= 10:
                        dist = 0
                    elif apparent_width >= 350:
                        dist = 10

                    if dist > 0 and time.time() - self.start_time > 3:
                        # print('Distance to mushroom is ', dist, ' apparent width is ', x2 - x1)
                        cv.putText(frame, 'Dist ' + str(dist) + ' cm', (x1, y1), cv.FONT_HERSHEY_SIMPLEX, 0.7,(255,255,255), 2)
                        self.last_mushroom_center = center
                        self.detected_once = True
                    self.last_mushroom_dist = dist

                # if debug:
                #     print(text_d + f" {round(float(row[4]),2)}")

        return frame
        

    def send_string_message(self, string_message):
        str_msg = String()
        str_msg.data = string_message
        self.motor_command_publisher.publish(str_msg)

    def grab_mushroom(self):
        self._logger.info('Grabbing object, distance is ' + str(self.last_mushroom_dist))
        self.send_string_message('s 1 70')
        time.sleep(1)
        self.send_string_message('s 0 110')

    def release_mushroom(self):
        self.send_string_message('s 0 130')        
        time.sleep(0.5)
        self.send_string_message('s 1 40')

    def move_command(self, x, alpha, speed = 0.01):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.z = alpha
        speed = speed
        self.cmd_vel_publisher.publish(msg)

    def stop_robot(self):
        print('stopping')
        self.move_command(0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ObjectDetectionNode()

    while minimal_subscriber.running:
        rclpy.spin_once(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()