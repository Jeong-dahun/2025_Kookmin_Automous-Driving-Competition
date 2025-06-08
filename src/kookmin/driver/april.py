#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#===================================================
# 자율주행 예선과제 - Jeong 
#===================================================
import rclpy
from rclpy.node import Node
import numpy as np
import cv2, os, math, time
import apriltag

#=============================================
# ROS2 Node 클래스 정의
#=============================================
class ARDriverNode(Node):
    def __init__(self):
        super().__init__('ar_driver')

        # 초기 변수 설정
        self.cv_image = None
        self.Blue = (0, 255, 0)
        self.Green = (0, 255, 0)
        self.Red = (0, 0, 255)
        self.Yellow = (0, 255, 255)
        self.Fix_Speed = 12
        self.new_angle = 0
        self.new_speed = self.Fix_Speed
        self.detector = apriltag.Detector()
        self.tag_size = 9.5
        self.camera_matrix = np.array([[371.42821, 0., 310.49805],
                                       [0., 372.60371, 235.74201],
                                       [0., 0., 1.]])

        self.get_logger().info("AR Driver Node Initialized")

    def ar_detect(self, image):
        ar_msg = {"ID": [], "DX": [], "DZ": []}
        if image is not None:
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            detections = self.detector.detect(gray_image)

            for detection in detections:
                for corner in detection.corners:
                    corner = tuple(map(int, corner))
                    cv2.circle(image, corner, 5, self.Green, 2)

                center = tuple(map(int, detection.center))
                cv2.circle(image, center, 5, self.Red, -1)
								
                tag_size_pixels = np.mean([
                    abs(detection.corners[0][1] - detection.corners[3][1]),
                    abs(detection.corners[1][1] - detection.corners[2][1])
                ])
                distance = ((self.camera_matrix[0, 0] * self.tag_size) / tag_size_pixels) * 100 / 126
                distance_to_tag_cm = distance * 1.1494 - 14.94
                x_offset_cm = ((center[0] - 320) * self.tag_size) / tag_size_pixels
                ar_msg["ID"].append(detection.tag_id)
                ar_msg["DZ"].append(distance_to_tag_cm)
                ar_msg["DX"].append(x_offset_cm)

        cv2.imshow('Detected AprilTags', image)
        cv2.waitKey(1)
        return ar_msg

    def check_AR(self, image):
        ar_data = self.ar_detect(image)
        if not ar_data["ID"]:
            return 99, 500, 500
        closest_index = ar_data["DZ"].index(min(ar_data["DZ"]))
        tag_id = ar_data["ID"][closest_index]
        z_pos = ar_data["DZ"][closest_index]
        x_pos = ar_data["DX"][closest_index]
        return tag_id, round(z_pos, 1), round(x_pos, 1)

