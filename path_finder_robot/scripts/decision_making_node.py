#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
import torch

debug = True

def detectx (frame, model):
    frame = [frame]
    results = model(frame)
    # results.show()
    # print( results.xyxyn[0])
    # print(results.xyxyn[0][:, -1])
    # print(results.xyxyn[0][:, :-1])

    labels, cordinates = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]

    return labels, cordinates


def plot_boxes(results, frame,classes):
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

            # if debug:
            #     print(text_d + f" {round(float(row[4]),2)}")

    return frame


class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('decision_making_node')
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self.camera_frame_received,
            10)
        self.camera_subscription
        self.br = CvBridge()
        self.running = True
        if debug:
            cv.namedWindow('Detections')
            cv.startWindowThread()

        self.model =  torch.hub.load('install/path_finder_robot/share/path_finder_robot/yolov5-deploy/yolov5', 'custom', source ='local', path='install/path_finder_robot/share/path_finder_robot/yolov5-deploy//mushroom.pt',force_reload=True) ### The repo is stored locally

        self.classes = self.model.names ### class names in string format  

    def camera_frame_received(self, msg):
        image = self.br.compressed_imgmsg_to_cv2(msg)
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        results = detectx(image, model=self.model)
        image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
        image = plot_boxes(results, image, classes=self.classes)
        if debug:
            cv.imshow('Detections', image)
            if cv.waitKey(1) & 0xFF == ord('q'):
                self.running = False

    

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ObjectDetectionNode()

    while minimal_subscriber.running:
        rclpy.spin_once(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()