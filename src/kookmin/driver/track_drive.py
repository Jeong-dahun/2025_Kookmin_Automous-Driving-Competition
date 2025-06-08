#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#===================================================
# 자율주행 예선과제 - Jeong
# TrackDriverNode: 카메라 및 LiDAR 데이터를 이용한
# 라바콘 주행 및 차선 유지 제어 구현
#===================================================

#=============================================
# 라이브러리 및 메시지 타입 임포트 블록
# - ROS 메시징, 센서 처리, 수치 계산, 시각화 등 필수 모듈
#=============================================
import rospy
from sensor_msgs.msg import Image, LaserScan
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge
import numpy as np
import cv2
import math, time, os
import matplotlib.pyplot as plt
import torch
#import sys

from hough import LaneDriverNode  # LANE DETECTION 코드 받아옴 
from traffic import TrafficSignCheckNode  # TRAFFICSIGN DETECT하는 코드 받아옴
import filter_pid 
from scipy.signal import medfilt

#=============================================
# TrackDriverNode 클래스 정의
# - ROS 노드 초기화, 콜백 함수, 제어 로직(main_loop) 을 포함
#=============================================
class TrackDriverNode:
    #=========================================
    # __init__ 메서드
    # - 노드 초기화 및 변수/모듈 설정
    # - Publisher/Subscriber, 시각화 윈도우 구성
    # - 센서 토픽 수신 대기 후 초기 정지 및 메인 루프 실행
    #=========================================
    def __init__(self):
        rospy.init_node('track_driver_node')

        # --- 주행 파라미터 초기화 --- 
        self.fix_speed = 15              # 기본 주행 속도
        self.new_angle = 0               # 제어할 조향 각도
        self.prev_angle = 0              # 이전 조향 각도
        self.new_speed = self.fix_speed  # 제어할 속도
        self.image = None                # 카메라 프레임 저장 변수
        self.motor_msg = XycarMotor()    # 모터 명령 메시지 객체
        self.ranges = None               # LiDAR 거리 배열 저장 변수
        self.bridge = CvBridge()         # OpenCV-ROS 이미지 변환기

        # --- 서브모듈 초기화 ---
        self.lane_driver = LaneDriverNode()                     # 차선 검출 노드
        self.traffic_driver = TrafficSignCheckNode()            # 신호등 인식 노드
        self.pid = filter_pid.PID(kp=0.3, ki=0.005, kd=0.15)    # 차선 PID 제어기
        self.cone_pid = filter_pid.PID(kp=8.0, ki=0.0, kd=0.5)  # LiDAR 기반 조향 PID

        # --- 라바콘 주행 상태 변수 --- 
        self.cone_detected_once = False  # 라바콘 주행 진입 여부 
        self.cone_clear_count = 0
        self.lane_change_start_time = None
        self.start_correction_done = False

        
        # --- ROS 통신 설정 ---
        self.motor_pub = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback, queue_size=1)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)

        # --- LiDAR 포인트 시각화 설정 ---
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_xlim(-120, 120)
        self.ax.set_ylim(-120, 120)
        self.ax.set_aspect('equal')
        self.lidar_points, = self.ax.plot([], [], 'bo')
        plt.ion()
        plt.show()

        # --- 센서 토픽 초기 수신 대기 ---
        rospy.wait_for_message("/usb_cam/image_raw", Image)
        rospy.loginfo("Camera Ready")
        rospy.wait_for_message("/scan", LaserScan)
        rospy.loginfo("Lidar Ready")

        # 초기 정지 후 메인 루프 진입
        self.stop_car(2)
        rospy.loginfo("Lane Driver Node Initialized")
        self.main_loop()

    #=========================================
    # img_callback 메서드
    # - 카메라 이미지 수신 시 호출
    # - ROS 이미지 메시지를 OpenCV BGR 포맷으로 변환하여 저장
    #=========================================
    def img_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    #=========================================
    # lidar_callback 메서드
    # - LiDAR 스캔 데이터 수신 시 호출
    # - 전체 360도 중 0~359 인덱스 범위 거리값 저장
    #=========================================
    def lidar_callback(self, msg):
        self.ranges = msg.ranges[0:360]

    #=========================================
    # drive 메서드
    # - 조향 각도(angle) 및 속도(speed)를
    #   모터 메시지에 할당 후 발행
    #=========================================
    def drive(self, angle, speed):
        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)
        self.motor_pub.publish(self.motor_msg)

    #=========================================
    # stop_car 메서드
    # - 지정된 duration(초) 동안 모터 정지 명령 반복
    #=========================================
    def stop_car(self, duration):
        rate = rospy.Rate(10)
        for _ in range(int(duration * 10)):
            self.drive(angle=0, speed=0)
            rate.sleep()

    #=========================================
    # visualize_lidar 메서드
    # - LiDAR 거리 데이터를 XY 평면에 실시간 플로팅하여
    #   주변 객체 분포 시각화 (디버깅용)
    #=========================================
    def visualize_lidar(self):
        if self.ranges is not None:
            angles = np.linspace(0, 2 * np.pi, len(self.ranges)) + np.pi / 2
            x = np.array(self.ranges) * np.cos(angles)
            y = np.array(self.ranges) * np.sin(angles)
            self.lidar_points.set_data(x, y)
            self.fig.canvas.draw_idle()
            plt.pause(0.01)
    
    #=========================================
    # steer_through_cones 메서드
    # - LiDAR 거리 데이터를 기반으로 노이즈 필터링
    # - 좌/우 자유 공간 기준 gap-finding 알고리즘 적용
    # - 최대 간격 중앙 방향으로 조향 각도 계산 반환
    #========================================= 
    def steer_through_cones(self):
        if self.ranges is None:
            return 0.0

        # 1) 노이즈 제거
        filt = medfilt(self.ranges, 3)

        # 2) 측면 영역: 왼쪽 210~350°, 오른쪽 10~150°
        left_sectors  = list(range(210, 355))
        right_sectors = list(range(5, 151))
        sectors = left_sectors + right_sectors

        # 3) 빈 공간(free space) 판정 (threshold 2.5m)
        free_thresh = 2.5
        free = [filt[ang] > free_thresh for ang in sectors]

        # 4) gap-finding 헬퍼
        def find_bearing(free_list, sector_list):
            gaps = []
            start = None
            for i, ok in enumerate(free_list):
                if ok and start is None:
                    start = i
                if not ok and start is not None:
                    gaps.append((start, i-1)); start = None
            if start is not None:
                gaps.append((start, len(free_list)-1))
            best = max(gaps, key=lambda g: g[1]-g[0])
            mid = (best[0] + best[1]) // 2
            sec = sector_list[mid]
            return sec if sec <= 180 else sec - 360

        # 5) 한쪽/양쪽 분기
        nL = len(left_sectors)
        left_free, right_free = free[:nL], free[nL:]
        small_gain = 3.0   # 한쪽만일 때
        large_gain = 4.0   # 양쪽 모두일 때
        bias = 0.0         # 센터링 바이어스

        if any(left_free) and not any(right_free):
            bearing = find_bearing(left_free, left_sectors)
            angle   = - (bearing - bias) * small_gain
            rospy.loginfo(f"[STEER_CONES] ← only left free: bearing {bearing:.1f}° → angle {angle:.1f}")

        elif any(right_free) and not any(left_free):
            bearing = find_bearing(right_free, right_sectors)
            angle   = - (bearing - bias) * small_gain
            rospy.loginfo(f"[STEER_CONES] → only right free: bearing {bearing:.1f}° → angle {angle:.1f}")

        else:
            # 양쪽 모두
            bearing = find_bearing(free, sectors)
            angle   = - (bearing - bias) * large_gain
            rospy.loginfo(f"[STEER_CONES] ↔ both free: bearing {bearing:.1f}° → angle {angle:.1f}")

        # 6) steering limit
        return float(np.clip(angle, -75, 75))

    #=========================================
    # lidar_drive 메서드
    # - steer_through_cones 래핑 함수
    #=========================================
    def lidar_drive(self):
        return self.steer_through_cones()



    #=========================================
    # main_loop 메서드
    # - 주행 모드(신호등, 차선, 라바콘, 추월) 제어 플로우 구현
    # - 모드별 분기 처리 및 상태 전환
    #=========================================
    # 1. traffic_sign 파악 
    # 2. lane detect 
    # 3. 라바콘 주행 (LiDAR로 라바콘 detect)
    # 4. 다시 lane detect 
    # 5. lane detect하면서 앞에 다른 차량 나타나면 detect 후 다른 차선으로 차선 변경하는 코드 

    def main_loop(self):
        rospy.loginfo("Track Driver starts...")
        # self.cam_exposure(110)

        TRAFFIC_SIGN, LANE_DRIVE_FIRST, SENSOR_DRIVE, LANE_DRIVE, OBJECT_DRIVE, FINISH = 1, 2, 3, 4, 5, 6
        drive_mode  = TRAFFIC_SIGN 
        # self.cam_exposure(98) 

        self.new_speed = self.fix_speed
        go_next = False
        count = 0
        View_Center = 300   # 원본 320 (300이 잘됨)
        self.stop_car(2.0)

        rospy.loginfo("==============================")
        rospy.loginfo(" S T A R T    D R I V I N G")
        rospy.loginfo("==============================")

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # --- MODE 1: TRAFFIC_SIGN ---
            # 이미지에서 파란불 신호등이 켜졌는지 확인하고,
            # 밝기가 충분히 높으면 운행 시작 모드로 전환
            if drive_mode == TRAFFIC_SIGN:
                if self.image is not None:
                    flag, color = self.traffic_driver.check_traffic_sign(self.image)
                    if flag and color == 'Blue' and self.traffic_driver.brightness_of_blue >= 100:
                        rospy.loginfo('#Traffic sign is Blue! -- Go Next!')
                        go_next = True

                if go_next:
                    self.stop_car(1.0)
                    cv2.destroyAllWindows()
                    drive_mode = LANE_DRIVE_FIRST
                    self.lane_change_start_time = rospy.Time.now()
                    # self.cam_exposure(98)
                    rospy.loginfo("----- Sensor driving 토대를 위한 LANE Driving1 starts... -----")
                    go_next = False
                    count = 0


            # --- MODE 2: LANE_DRIVE_FIRST ---
            # 카메라 차선 검출로 초기 차선 주행을 수행하고,
            # LiDAR 전방 거리(front)가 5m 이하로 가까워지면 센서 모드로 전환
            elif drive_mode == LANE_DRIVE_FIRST:
                if self.image is not None:
                        found, x_left, x_right = self.lane_driver.lane_detect(self.image)
                        if found:
                            x_mid = (x_left + x_right) // 2
                            self.new_angle = (x_mid - View_Center) * 0.7
                            self.new_angle = self.pid.pid_control(self.new_angle)
                            max_steer_change = 10
                            angle_diff = self.new_angle - self.motor_msg.angle
                            if abs(angle_diff) > max_steer_change:
                                self.new_angle = self.motor_msg.angle + np.clip(angle_diff, -max_steer_change, max_steer_change)
                            if abs(self.new_angle) > 30:
                                self.new_speed = 20
                            elif abs(self.new_angle) > 15:
                                self.new_speed = 30
                            else:
                                self.new_speed = 40
                            self.drive(self.new_angle, self.new_speed)
                        else:
                            # 차선 인식 실패 시 이전 각도로 유지
                            self.drive(self.prev_angle, self.new_speed)

                    # ③ 라바콘 감지 조건
                if self.ranges is not None:
                    front = np.min(self.ranges[340:360] + self.ranges[0:20])
                    if front < 5.0: # 원래 5.0
                        rospy.loginfo("[LANE_CHANGE_RIGHT] Cone detected ahead. Switching to SENSOR_DRIVE.")
                        self.stop_car(1.0)
                        drive_mode = SENSOR_DRIVE
                        self.cone_detected_once = True
                        self.cone_clear_count = 0


            # --- MODE 3: SENSOR_DRIVE ---
            # LiDAR 기반으로 라바콘 검출 및 통과 주행 수행
            elif drive_mode == SENSOR_DRIVE:
                rospy.loginfo("[SENSOR_DRIVE] Warming up LiDAR...")
                if self.ranges is not None:
                    # Percentile로 잡음 제거한 거리값

                    # 시계방향 기준 (0도 기준, 360개의 레이저 데이터)
                    # - front: 350~359 + 0~10 (정면)
                    # - right: 70~110 (오른쪽)
                    # - left: 250~290 (왼쪽)
                    front = np.percentile(np.concatenate((self.ranges[350:360], self.ranges[0:10])), 10)
                    left_front = np.percentile(self.ranges[250:290], 40)
                    right_front = np.percentile(self.ranges[70:110], 40)
                    left_side = np.percentile(self.ranges[265:275], 20)
                    right_side = np.percentile(self.ranges[85:95], 20)

                    rospy.loginfo(f"[SENSOR_DRIVE] front={front:.2f}, left={left_front:.2f}, right={right_front:.2f}, "
                                  f"left_side={left_side:.2f}, right_side={right_side:.2f}")

                    if self.cone_detected_once:
                        angle = self.lidar_drive()
                        self.new_angle = angle
                        self.new_speed = 8
                        self.drive(self.new_angle, self.new_speed)

                        self.cone_clear_count += 1
                        rospy.loginfo(f"[SENSOR_DRIVE] Driving through cones... Clear count = {self.cone_clear_count}")
                        if self.cone_clear_count >= 420:
                            rospy.loginfo("[SENSOR_DRIVE] Cone passed. Switching to lane driving.")
                            go_next = True

                    else:
                        if front < 6.0: # or left_front < 7.0 or right_front < 7.0 or left_side < 2.5 or right_side < 2.5
                            rospy.loginfo("[SENSOR_DRIVE] Cone detected. Starting cone drive.")
                            rospy.loginfo(f"[SENSOR_DRIVE] front={front:.2f}, left_front={left_front:.2f}, right_front={right_front:.2f}, left_side={left_side:.2f}, right_side={right_side:.2f}")
                            self.cone_detected_once = True
                            self.cone_clear_count = 0

                if go_next:
                    self.stop_car(1.0)
                    cv2.destroyAllWindows()
                    drive_mode = LANE_DRIVE
                    rospy.loginfo("----- LANE DRIVING starts... -----")
                    go_next = False
                    count = 0


            # --- MODE 4: LANE_DRIVE ---
            # 카메라 차선 검출로 차선 유지 주행
            # 전방 느린 차 감지 시 OBJECT_DRIVE로 전환
            elif drive_mode == LANE_DRIVE:
                if self.image is not None:
                    found, x_left, x_right = self.lane_driver.lane_detect(self.image)
                    if found:
                        x_midpoint = (x_left + x_right) // 2
                        self.new_angle = (x_midpoint - View_Center) * 0.7
                        self.new_angle = self.pid.pid_control(self.new_angle)
                        # 이전 각도와 비교해서 급격한 변화 제한
                        max_steer_change = 10  # 최대 10도 이상 변화 금지
                        angle_diff = self.new_angle - self.motor_msg.angle
                        if abs(angle_diff) > max_steer_change:
                            self.new_angle = self.motor_msg.angle + np.clip(angle_diff, -max_steer_change, max_steer_change)
                            
                            self.prev_angle = self.new_angle
                            
                        if abs(self.new_angle) > 30:
                            self.new_speed = 20
                        elif abs(self.new_angle) > 15:
                            self.new_speed = 30
                        else:
                            self.new_speed = 40
                        rospy.loginfo(f"Angle={self.new_angle:.1f} Speed={self.new_speed:.1f}")
                        self.drive(self.new_angle, self.new_speed)
                    else:
                        rospy.loginfo(f"Lane finding fails... Keep going! {count}")
                        count += 1
                        self.new_angle = self.prev_angle
                        self.drive(self.new_angle, self.new_speed)

                
                #LANE_DRIVE -> SENSOR_DRIVE 갈 코드 
                if self.ranges is not None:
                    front = np.min(self.ranges[340:360] + self.ranges[0:20])
                    if front < 10.0:  # 거리 조건은 상황에 따라 튜닝
                        rospy.loginfo("[LANE_DRIVE] Slow vehicle ahead! Switching to OBJECT_DRVIE!")
                        #self.stop_car(1.0)
                        self.overtake_stage = 0
                        self.overtake_direction = None
                        drive_mode = OBJECT_DRIVE
                        rospy.loginfo("----- Object driving starts... -----")
                
            # --- MODE 5: OBJECT_DRIVE ---
            # 타이머 기반 단순 추월: 3단계(추월 조향 → PID 주행 → 복귀 조향)
            elif drive_mode == OBJECT_DRIVE:
                # ------------------------------------------------
                # OBJECT_DRIVE: 단순 타이머 기반 추월 & 복귀
                # ------------------------------------------------
                # 스테이지 정의
                # 0 = 대기(아직 추월 전)
                # 1 = 좌측(또는 우측) 추월 조향 중
                # 2 = 새 차선에서 PID 차선 추종
                # 3 = 복귀 조향 중
                if not hasattr(self, 'overtake_stage'):
                    self.overtake_stage = 0

                # 1) 전방 거리 읽어오기
                front = np.percentile(
                np.concatenate((self.ranges[350:360], self.ranges[0:10])),
                10  # 10% 퍼센타일: 노이즈에 강하면서도 조기 감지
                )           
                now = rospy.Time.now()
                TH = 8.0  # 감지 임계 거리

                # ----------------------------
                # stage 0: 전방 차량 감지되면 추월 시작
                # ----------------------------
                if self.overtake_stage == 0 and front < TH:
                    self.overtake_stage = 1
                    self.overtake_start_time = now
                    rospy.loginfo("[OBJECT_DRIVE] 차량 감지 → 추월 조향 시작")

                # ----------------------------
                # stage 1: 2.5초 동안 고정 조향 (추월 차선)
                # ----------------------------
                if self.overtake_stage == 1:
                    elapsed = (now - self.overtake_start_time).to_sec()
                    if elapsed < 2.5:
                        # left 우선, front<TH 시 좌측으로 추월
                        # front<TH 두 번 감지 시 우측 복귀(아래 stage3에서 처리)
                        self.drive(-20, 40)
                    else:
                        self.overtake_stage = 2
                        self.return_enable_time = now + rospy.Duration(0.5)  # 1초 뒤부터 복귀 허용
                        rospy.loginfo("[OBJECT_DRIVE] 추월 조향 완료 → 차선 추종 모드")

                # ----------------------------
                # stage 2: 새 차선에서 PID 차선 추종
                # ----------------------------
                elif self.overtake_stage == 2:
        
                    if self.image is not None:
                        found, x_left, x_right = self.lane_driver.lane_detect(self.image)
                        if found:
                            x_midpoint = (x_left + x_right) // 2
                            self.new_angle = (x_midpoint - View_Center) * 0.7
                            self.new_angle = self.pid.pid_control(self.new_angle)
                            # 급격한 변화 제한
                            max_delta = 10
                            delta = self.new_angle - self.motor_msg.angle
                            if abs(delta) > max_delta:
                                self.new_angle = self.motor_msg.angle + np.clip(delta, -max_delta, max_delta)
                            # 속도 조절
                            if abs(self.new_angle) > 30:
                                self.new_speed = 20
                            elif abs(self.new_angle) > 15:
                                self.new_speed = 45
                            else:
                                self.new_speed = 55
                            rospy.loginfo(f"[OBJECT_DRIVE·STG2] Angle={self.new_angle:.1f} Speed={self.new_speed:.1f}")
                            self.drive(self.new_angle, self.new_speed)
                        else:
                            # 차선 인식 실패 시 이전 각도로 유지
                            rospy.loginfo("[OBJECT_DRIVE·STG2] Lane finding fails... 유지")
                            self.drive(self.prev_angle, self.new_speed)
            
                    # 복귀 잠금 체크 후, front < 5 이면 다음 스테이지
                    if front < 11.0 and now >= self.return_enable_time:
                        self.overtake_stage = 3
                        self.overtake_start_time = now
                        rospy.loginfo("[OBJECT_DRIVE] 새 차선에서 차량 재감지 → 복귀 조향 시작")
            

                # ----------------------------
                # stage 3: 2.5초 동안 반대 방향 조향 (복귀)
                # ----------------------------
                elif self.overtake_stage == 3:
                    elapsed = (now - self.overtake_start_time).to_sec()
                    if elapsed < 2.5:
                        self.drive(+20, 35)
                    else:
                        # 복귀 완료 → 다시 LANE_DRIVE 로 돌아감
                        drive_mode = LANE_DRIVE
                        self.overtake_stage = 0
                        rospy.loginfo("[OBJECT_DRIVE] 복귀 완료 → LANE_DRIVE 모드")

            #self.visualize_lidar()
            # self.drive(angle=0.0, speed=self.fix_speed)
            rate.sleep()

            
#=============================================
# 메인 실행 블록
# - TrackDriverNode 인스턴스 생성 후
#   ROS 인터럽트 처리
#=============================================
if __name__ == '__main__':
    try:
        TrackDriverNode()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()