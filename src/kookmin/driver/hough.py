#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#=============================================
# Jeong 
# ============================================

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import math

#=============================================
# 차선인식 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30  # 카메라 FPS 초당 30장의 사진을 보냄
WIDTH = 640  # 카메라 이미지 가로x세로 크기
HEIGHT = 480  # 카메라 이미지 가로x세로 크기
ROI_START_ROW = 270  # 차선을 찾을 ROI 영역의 시작 Row값
ROI_END_ROW = 460  # 차선을 찾을 ROI 영역의 끝 Row값
ROI_HEIGHT = ROI_END_ROW - ROI_START_ROW  # ROI 영역의 세로 크기  
L_ROW = 30  # 차선의 위치를 찾기 위한 ROI 안에서의 기준 Row값 
Blue =  (255,0,0) # 파란색
Green = (0,255,0) # 녹색
Red =   (0,0,255) # 빨간색
Yellow = (0,255,255) # 노란색
View_Center = WIDTH//2  # 화면의 중앙값 = 카메라 위치

#=============================================
# ROS1 Node 클래스 정의
#=============================================
class LaneDriverNode:
    def __init__(self):
        # ROS 노드 초기화
        # rospy.init_node('lane_detection_node', anonymous=False)

        # 클래스 속성 초기화
        self.bridge = CvBridge()
        self.prev_x_left = 0
        self.prev_x_right = 0

#        # 카메라 이미지 구독자 등록
#        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
#
#    def image_callback(self, msg):
#        # ROS 이미지 메시지를 OpenCV 이미지로 변환
#        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#        found, x_left, x_right = self.lane_detect(image)
#        if found:
#            rospy.loginfo(f"Lane detected! x_left: {x_left}, x_right: {x_right}")
#        else:
#            rospy.loginfo("Lane not found")

    #=============================================
    # 카메라 이미지에서 차선을 찾아 그 위치를 반환하는 함수
    #=============================================
    def lane_detect(self, image):
        img = image.copy()  # 이미지처리를 위한 카메라 원본이미지 저장
        display_img = img  # 디버깅을 위한 디스플레이용 이미지 저장

        # img(원본이미지)의 특정영역(ROI Area)을 잘라내기
        roi_img = img[ROI_START_ROW:ROI_END_ROW, 0:WIDTH]
        line_draw_img = roi_img.copy()

        # 원본 칼라이미지를 그레이 회색톤 이미지로 변환하고 
        # 블러링 처리를 통해 노이즈를 제거한 후에 
        # Canny 변환을 통해 외곽선 이미지로 만들기
        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edge_img = cv2.Canny(np.uint8(blur_gray), 60, 75)

        # 사다리꼴 마스크 생성 및 적용
        mask = np.zeros_like(edge_img)
        polygon = np.array([[
            (int(WIDTH * 0.1), ROI_HEIGHT),       # 좌하
            (int(WIDTH * 0.3), 0),                # 좌상
            (int(WIDTH * 0.65), 0),                # 우상
            (int(WIDTH * 0.9), ROI_HEIGHT)        # 우하
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_edges = cv2.bitwise_and(edge_img, mask)

        # 디버깅용 Canny 이미지 확인
        cv2.imshow("Lane Detection Canny Image", masked_edges)

        # 잘라낸 이미지에서 HoughLinesP 함수를 사용하여 선분들을 찾음
        all_lines = cv2.HoughLinesP(masked_edges, 1, math.pi / 180, 50, 50, 20)
        if all_lines is None:
            cv2.imshow("Lanes positions", display_img)
            cv2.waitKey(1)
            return False, 0, 0

        # 선분들의 기울기 값을 각각 모두 구한 후에 리스트에 담음
        # 기울기의 절대값이 너무 작은 경우 (수평선에 가까운 경우) 제외
        slopes = []
        filtered_lines = []
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            slope = float('inf') if (x2 == x1) else float(y2 - y1) / float(x2 - x1)
            if 0.2 < abs(slope):
                slopes.append(slope)
                filtered_lines.append(line[0])

        if not filtered_lines:
            cv2.imshow("Lanes positions", display_img)
            cv2.waitKey(1)
            return False, 0, 0

        # 왼쪽과 오른쪽 차선을 나누어 담기
        left_lines, right_lines = [], []
        for j in range(len(slopes)):
            x1, y1, x2, y2 = filtered_lines[j]
            slope = slopes[j]
            if slope < 0 and x2 < WIDTH / 2:
                left_lines.append([x1, y1, x2, y2])
            elif slope > 0 and x1 > WIDTH / 2:
                right_lines.append([x1, y1, x2, y2])

        #=============================================
        # 왼쪽/오른쪽 차선의 대표직선을 계산하는 함수
        #=============================================
        def get_line_params(lines):
            if not lines:
                return 0.0, 0.0
            x_sum = y_sum = m_sum = 0.0
            for x1, y1, x2, y2 in lines:
                x_sum += x1 + x2
                y_sum += y1 + y2
                m_sum += 0 if x2 == x1 else float(y2 - y1) / float(x2 - x1)
            x_avg = x_sum / (2 * len(lines))
            y_avg = y_sum / (2 * len(lines))
            m_avg = m_sum / len(lines)
            b = y_avg - m_avg * x_avg
            return m_avg, b

        # 왼쪽/오른쪽 대표직선 계산
        m_left, b_left = get_line_params(left_lines)
        m_right, b_right = get_line_params(right_lines)

        # 기준 수평선(L_ROW)과 대표직선의 교점 구하기
        x_left = self.prev_x_left if m_left == 0.0 else int((L_ROW - b_left) / m_left)
        x_right = self.prev_x_right if m_right == 0.0 else int((L_ROW - b_right) / m_right)

        # 한쪽 차선이 없을 경우 반대쪽 정보로 보정
        if m_left == 0.0 and m_right != 0.0:
            x_left = x_right - 380
        if m_right == 0.0 and m_left != 0.0:
            x_right = x_left + 380

        # 이전 좌표 저장
        self.prev_x_left = x_left
        self.prev_x_right = x_right

        # 중앙점 계산
        x_midpoint = (x_left + x_right) // 2

        # 디버깅용 이미지 그리기
        cv2.line(line_draw_img, (0, L_ROW), (WIDTH, L_ROW), Yellow, 2)
        cv2.rectangle(line_draw_img, (x_left - 5, L_ROW - 5), (x_left + 5, L_ROW + 5), Green, 4)
        cv2.rectangle(line_draw_img, (x_right - 5, L_ROW - 5), (x_right + 5, L_ROW + 5), Green, 4)
        cv2.rectangle(line_draw_img, (x_midpoint - 5, L_ROW - 5), (x_midpoint + 5, L_ROW + 5), Blue, 4)
        cv2.rectangle(line_draw_img, (View_Center - 5, L_ROW - 5), (View_Center + 5, L_ROW + 5), Red, 4)

        # ROI 영역에 결과 덮어쓰기 후 디스플레이
        display_img[ROI_START_ROW:ROI_END_ROW, 0:WIDTH] = line_draw_img
        cv2.imshow("Lanes positions", display_img)
        cv2.waitKey(1)

        return True, x_left, x_right


#=============================================
# 메인 함수 정의
#=============================================
if __name__ == '__main__':
    try:
        node = LaneDriverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
