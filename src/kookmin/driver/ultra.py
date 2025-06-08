#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#===================================================
# 자율주행 예선과제 - Jeong 
#===================================================
import rospy

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
class UltraDriverNode:
    def __init__(self):      
        rospy.loginfo("Ultrasonic Driver Node Initialized")
                
    def sonic_drive(self, ultra_msg):
        # 초음파 센서 데이터를 기반으로 조향각 계산 함수
        if ultra_msg[1] > ultra_msg[3]:
            angle = -(ultra_msg[1]-ultra_msg[3])*2
            #angle = -50  # 왼쪽 벽이 더 가까운 경우 왼쪽으로 조향
        elif ultra_msg[1] < ultra_msg[3]:
            angle = (ultra_msg[3]-ultra_msg[1])*2
            #angle = 50  # 오른쪽 벽이 더 가까운 경우 오른쪽으로 조향
        else:
            angle = 0  # 벽이 동일한 거리인 경우 직진
        return angle