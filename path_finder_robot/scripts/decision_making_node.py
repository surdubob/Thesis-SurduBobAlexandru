#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
import torch

debug = False


class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('decision_making_node')
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self.camera_frame_received,
            10)
        self.camera_depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_frame_received,
            10)
        
        self.camera_subscription
        # self.camera_depth_subscription
        self.last_depth_array = np.empty((1000, 1000))

        self.br = CvBridge()
        self.running = True

        self.processed_publisher = self.create_publisher(CompressedImage,
                                                         '/mushroom_detection_image/compressed',
                                                         10)
        
        self.processed_depth_publisher = self.create_publisher(Image,
                                                         '/mushroom_detection_image/depth',
                                                         10)
        if debug:
            cv.namedWindow('Detections')
            cv.startWindowThread()

        self.model =  torch.hub.load('install/path_finder_robot/share/path_finder_robot/yolov5-deploy/yolov5', 'custom', source ='local', path='install/path_finder_robot/share/path_finder_robot/yolov5-deploy//mushroom.pt',force_reload=True) ### The repo is stored locally

        self.classes = self.model.names ### class names in string format  

    def camera_frame_received(self, msg):
        image = self.br.compressed_imgmsg_to_cv2(msg)
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        results = self.detectx(image, model=self.model)
        image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
        image = self.plot_boxes(results, image, classes=self.classes)
        
        # print('color shape: ', image.shape)

        pbls = CompressedImage()
        pbls.format = "jpeg"
        pbls.data = cv.imencode('.jpg', image)[1].tobytes()
        
        self.processed_publisher.publish(pbls)

        dpth = self.br.cv2_to_imgmsg(self.last_depth_array)
        self.processed_depth_publisher.publish(dpth)

        if debug:
            cv.imshow('Detections', image)
            if cv.waitKey(1) & 0xFF == ord('q'):
                self.running = False

    def depth_frame_received(self, ros_image):
        depth_image = self.br.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)[80:400,144:704]
        depth_array = cv.resize(depth_array, (640, 480))
        # center_idx = [int(np.array(depth_array.shape)[0] / 2), int(np.array(depth_array.shape)[1] / 2)]
        self.last_depth_array = depth_array
        a = 0
        for i in range(10000):
            a += 1
        # print('depth shape: ', depth_array.shape)
        # print ('center depth:', depth_array[center_idx[0], center_idx[1]])


    def detectx (self, frame, model):
        frame = [frame]
        results = model(frame)
        # results.show()
        # print( results.xyxyn[0])
        # print(results.xyxyn[0][:, -1])
        # print(results.xyxyn[0][:, :-1])

        labels, cordinates = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]

        return labels, cordinates


    def plot_boxes(self, results, frame, classes):
        labels, cord = results
        n = len(labels)
        x_shape, y_shape = frame.shape[1], frame.shape[0]

        ### looping through the detections
        for i in range(n):
            row = cord[i]
            if row[4] >= 0.55: ### threshold value for detection. We are discarding everything below this value
                x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape) ## BBOx coordniates
                text_d = classes[int(labels[i])]

                if text_d == 'mushroom':
                    cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2) ## BBox
                    cv.rectangle(frame, (x1, y1-20), (x2, y1), (0, 255,0), -1) ## for text label background

                    cv.putText(frame, text_d + f" {round(float(row[4]),2)}", (x1, y1), cv.FONT_HERSHEY_SIMPLEX, 0.7,(255,255,255), 2)

                    # print('x1: ', x1, '            x2: ', x2)

                    dist = self.last_depth_array[int((y1 + y2) / 2), int((x1 + x2) / 2)] / 10
                    # cv.circle(self.last_depth_array, (int((x1 + x2) / 2), int((y1 + y2) / 2)), 10, 255, -1)
                    cv.circle(frame, (int((x1 + x2) / 2), int((y1 + y2) / 2)), 3, (255, 0, 0), -1)
                    if dist > 0:
                        print('Distance to mushroom is ', dist)

                # if debug:
                #     print(text_d + f" {round(float(row[4]),2)}")

        return frame
        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ObjectDetectionNode()

    while minimal_subscriber.running:
        rclpy.spin_once(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()