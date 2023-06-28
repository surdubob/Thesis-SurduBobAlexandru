#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose, Point, PoseStamped
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException
from action_msgs.srv import CancelGoal
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

import time

import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
import torch
from scipy.interpolate import interp1d
from enum import Enum
import math
import os

from path_finder_robot.srv import ChangeState

from threading import Timer, Thread

debug = False

class RobotState(Enum):
    NAVIGATION = 1
    FINDING = 2
    APPROACHING = 3
    CLOSE = 4
    GRABBING = 5
    GRABBED = 6


class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('decision_making_node')
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self.camera_frame_received,
            10)
        
        # qos_sensor = QoSProfile(
        #     reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        #     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        #     depth=1
        # )
        # self.pose_subscription = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_received, qos_sensor)
        self.tf_subscription = self.create_subscription(TFMessage, '/tf', self.tf_received, 10)
        
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.decision_making_state = RobotState.NAVIGATION

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
        self.last_pose = None
        self.objects_array = []
        self.pick_object_index = None
        self.last_navigation_command = 0

        self.processed_publisher = self.create_publisher(CompressedImage,
                                                         '/object_detection_image/compressed',
                                                         10)
        
        self.motor_command_publisher = self.create_publisher(String, '/arduino_raw_commands', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_decision_making', 10)
        self.object_publisher = self.create_publisher(MarkerArray, '/detected_object_markers', 10)
        self.pose_transform_publisher = self.create_publisher(TFMessage, '/pose_transforms', 10)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # self.cancel_goal_publisher = self.create_client(CancelGoal, '/', 10)
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        time.sleep(0.5)
        self.release_mushroom()
        
        if debug:
            cv.namedWindow('Detections')
            cv.startWindowThread()

        self.model =  torch.hub.load('install/path_finder_robot/share/path_finder_robot/yolov5-deploy/yolov5', 'custom', source ='local', path='install/path_finder_robot/share/path_finder_robot/yolov5-deploy//mushroom.pt',force_reload=True) ### The repo is stored locally

        self.srv = self.create_service(ChangeState, 'change_state', self.change_decision_making_state)

        self.decision_making_thread = Thread(target=self.decision_making_function)
        self.decision_making_thread.daemon = True
        self.decision_making_thread.start()

        self.classes = self.model.names ### class names in string format  
        self.start_time = time.time()
        self.release_mushroom()

    def camera_frame_received(self, msg):
        # print('new frame yay')
        image = self.br.compressed_imgmsg_to_cv2(msg)
        self.frame_width = image.shape[1]
        self.frame_height = image.shape[0]
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        results = self.detectx(image, model=self.model)
        image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
        image = self.plot_boxes_and_calculate_distance(results, image, classes=self.classes)
        
        # cv.line(image, (int(self.frame_width / 2 + 70), 0), (int(self.frame_width / 2 + 70), self.frame_width), (255, 0, 0), 3)
        # cv.line(image, (int(self.frame_width / 2 + 140), 0), (int(self.frame_width / 2 + 140), self.frame_width), (255, 0, 0), 3)

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
        try:
            while rclpy.ok():
                if self.pick_object_index is not None:
                    # print(self.last_mushroom_center, self.last_mushroom_dist)
                    if self.last_results is not None:
                        labels, cord = self.last_results
                        if self.detected_once:
                            if self.decision_making_state == RobotState.FINDING:
                                if len(labels) > 0 and cord[0][4] > 0.7:
                                    # self.last_mushroom_dist = self.objects_array[self.pick_object_index][1]
                                    # self.last_mushroom_center = self.objects_array[self.pick_object_index][2]
                                    object_coordinates = self.calculate_coordinates_of_detected_object(self.last_mushroom_center, self.last_mushroom_dist)
                                    print(self.objects_array[self.pick_object_index])
                                    if object_coordinates is not None:
                                        if abs(self.objects_array[self.pick_object_index][0].x - object_coordinates.x) < 1 and abs(self.objects_array[self.pick_object_index][0].y - object_coordinates.y) < 1:
                                            self.decision_making_state = RobotState.APPROACHING
                                            self.cancelNavigation()
                                        else:
                                            self.navigateToPose(object_coordinates)
                                else:
                                    self.navigateToPose(self.objects_array[self.pick_object_index][0])
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
                                    self.move_command(0.0, angle * 3.0, speed=0.5)
                                    time.sleep(0.7)
                                    self.move_command(0.0, 0.0, speed=0.000)
                                    time.sleep(0.4)

                            if self.decision_making_state == RobotState.GRABBING:
                                # print(self.last_mushroom_dist)
                                if self.last_mushroom_dist > 11 and len(labels) > 0 and cord[0][4] > 0.7:
                                    self.move_command(0.1, 0.0, speed=0.005)
                                elif len(labels) > 0 and cord[0][4] > 0.7:
                                    self.grab_mushroom()
                                    self.decision_making_state = RobotState.GRABBED
                            if self.decision_making_state == RobotState.GRABBED:
                                pass
                else:
                    self.decision_making_state = RobotState.NAVIGATION
                time.sleep(0.02)
        except (KeyboardInterrupt, SystemExit):
            exit()

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
                        object_coordinates = self.calculate_coordinates_of_detected_object(center, dist)
                        if object_coordinates:
                            gasit = False
                            gasit_index = 0
                            for ind, obj in enumerate(self.objects_array):
                                if abs(obj[0].x - object_coordinates.x) < 1 and abs(obj[0].y - object_coordinates.y) < 1:
                                    gasit = True
                                    gasit_index = ind
                            if not gasit and self.decision_making_state == RobotState.NAVIGATION and self.last_pose:
                                if (abs(self.last_pose.translation.x - object_coordinates.x) > 0.3 or \
                                    abs(self.last_pose.translation.y - object_coordinates.y) > 0.3) and \
                                    (abs(x1-x2) - abs(y1-y2)) < 30 and (120 < center[0] < 520):
                                    # print('intra')
                                    self.objects_array.append([object_coordinates, dist, center])
                        
                    self.last_mushroom_dist = dist

        object_list_message = MarkerArray()
        for obj in self.objects_array:
            object_list_message.markers.append(self.create_marker_for_object(obj[0]))
        self.object_publisher.publish(object_list_message)

        return frame
        
    def create_marker_for_object(self, coords=None, name="Mushroom"):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = 8
        marker.id = 0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.text = name

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose = Pose()
        # Set the pose of the marker
        marker.pose.position.x = coords.x if coords != None else 0.0
        marker.pose.position.y = coords.y if coords != None else 0.0
        marker.pose.position.z = coords.z if coords != None else 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        return marker
    
    def eulerFromQuat(self, quat):
        q0 = quat.x
        q1 = quat.y
        q2 = quat.z
        q3 = quat.w

        Rx = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - (2 * (q1 * q1 + q2 * q2)))
        Ry = math.asin(2 * (q0 * q2 - q3 * q1))
        Rz = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - (2  * (q2 * q2 + q3 * q3)))

        euler = [Rx, Ry, Rz]

        return euler
			

    def calculate_coordinates_of_detected_object(self, object_center, distance):
        object_point = Point()
        
        # robot_angles = self.eulerFromQuat(self.last_pose.orientation)
        try:
            trans = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            last_pose = trans.transform

            robot_angles = self.eulerFromQuat(last_pose.rotation)
            # print(math.atan((self.frame_width / 2 - object_center[0]) / distance / 20))
            # object_point.x = self.last_pose.position.x + distance / 100 * math.cos(robot_angles[0] + math.atan((self.frame_width / 2 - object_center[0]) / distance / 20))
            # object_point.y = self.last_pose.position.y + distance / 100 * math.sin(robot_angles[0] + math.atan((self.frame_width / 2 - object_center[0]) / distance / 20))
            
            object_point.x = last_pose.translation.x + distance / 100 * math.cos(robot_angles[0] + math.atan((self.frame_width / 2 - object_center[0]) / distance / 20))
            object_point.y = last_pose.translation.y + distance / 100 * math.sin(robot_angles[0] + math.atan((self.frame_width / 2 - object_center[0]) / distance / 20))
            object_point.z = 0.0
            # print(object_point)
            return object_point
        except Exception as e:
            # print(e)
            pass
        
        return None
        
    

    def tf_received(self, msg):
        # for tr in msg.transforms:
        #     if tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_link':
        #         self.last_pose = tr.transform
        try:
            trans = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            mesaj = TFMessage()
            mesaj.transforms.append(trans)
            self.pose_transform_publisher.publish(mesaj)
            self.last_pose = trans.transform
        except Exception as e:
            # print(e)
            pass
        

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

    def change_decision_making_state(self, request, response):
        print('I was called with: ' + request.state_name)
        if request.state_name.startswith('pick_object'):
            print('Going to pick object', int(request.state_name[request.state_name.index(' '):]))
            self.pick_object_index = int(request.state_name[request.state_name.index(' '):])
            self.decision_making_state = RobotState.FINDING
        elif request.state_name == 'release':
            self.release_mushroom()
        response.response_data = request.state_name
        return response

    def navigateToPose(self, coords):
        goal_pose = NavigateToPose.Goal()
        
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = coords.x
        goal_pose.pose.pose.position.y = coords.y
        # goal_pose.pose.pose.position.z = 0.0
        # goal_pose.pose.pose.orientation.x = 0.0
        # goal_pose.pose.pose.orientation.y = 0.0
        # goal_pose.pose.pose.orientation.z = 0.0
        goal_pose.pose.pose.orientation.w = 1.0

        if time.time() - self.last_navigation_command > 1000:
            print('Sending navigation goal')
            self.last_navigation_command = time.time()
            # self.goal_pose_publisher.publish(goal_pose)
            self.action_client.send_goal_async(goal_pose)

    def cancelNavigation(self):
        self.action_client.wait_for_server()
        print('Cancelling navigation goal')
        # Cancel the navigation goal
        # cancel_request = self.action_client.create_goal_cancel_request()
        # self.action_client.send_goal_cancel_request(cancel_request)
        active_goals = self.action_client._goal_handles

        # Cancel each goal handle
        for goal_handle in active_goals:
            self.action_client._cancel_goal_async(goal_handle)
            

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ObjectDetectionNode()

    while minimal_subscriber.running:
        rclpy.spin_once(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
