#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#=============================================
# 자율주행 예선과제 - jeong
#=============================================
import numpy as np

#=============================================
# 이동평균필터 클래스
#=============================================
class MovingAverage:

    # 클래스 생성과 초기화 함수 (데이터의 개수를 지정)
    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = list(range(1, n + 1))

    # 새로운 샘플 데이터를 추가하는 함수
    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data.pop(0)  # 가장 오래된 샘플 제거
            self.data.append(new_sample)

    # 저장된 샘플 데이터의 갯수를 구하는 함수
    def get_sample_count(self):
        return len(self.data)

    # 이동평균값을 구하는 함수
    def get_mavg(self):
        if not self.data:
            return 0.0
        return float(sum(self.data)) / len(self.data)

    # 중앙값을 사용해서 이동평균값을 구하는 함수
    def get_mmed(self):
        if not self.data:
            return 0.0
        return float(np.median(self.data))

    # 가중치를 적용하여 이동평균값을 구하는 함수        
    def get_wmavg(self):
        if not self.data:
            return 0.0
        s = sum(x * w for x, w in zip(self.data, self.weights[:len(self.data)]))
        return float(s) / sum(self.weights[:len(self.data)])

    # 샘플 데이터 중에서 제일 작은 값을 반환하는 함수
    def get_min(self):
        if not self.data:
            return 0.0
        return float(min(self.data))
    
    # 샘플 데이터 중에서 제일 큰 값을 반환하는 함수
    def get_max(self):
        if not self.data:
            return 0.0
        return float(max(self.data))

#=============================================
# 프로그램에서 사용할 PID 클래스
#=============================================  
class PID:

    # 클래스 생성 초기화 함수 (게인값과 오차값 초기화)
    def __init__(self, kp, ki, kd):
        # PID 게인 초기화
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        # 이전 오차 초기화
        self.cte_prev = None  # None으로 설정하여 초기 상태 확인
        # 각 오차 초기화
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0
        # 적분 오차 제한값 설정
        self.i_min = -10
        self.i_max = 10

    def pid_control(self, cte, dt=1.0):
        # 초기 호출 시 이전 오차를 현재 오차로 설정
        if self.cte_prev is None:
            self.cte_prev = cte

        # 미분 오차 계산 (dt 고려)
        self.d_error = (cte - self.cte_prev) / dt

        # 비례 오차 계산
        self.p_error = cte

        # 적분 오차 계산 및 제한 적용
        self.i_error += cte * dt
        self.i_error = max(min(self.i_error, self.i_max), self.i_min)

        # 이전 오차 업데이트
        self.cte_prev = cte

        # PID 제어 출력 계산
        return self.Kp * self.p_error + self.Ki * self.i_error + self.Kd * self.d_error
