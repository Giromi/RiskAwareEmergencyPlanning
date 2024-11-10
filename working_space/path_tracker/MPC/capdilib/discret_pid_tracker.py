#!/usr/bin/env python

import numpy as np
# u(k ) = u(k - 1 )

class DiscretPIDController:
    def __init__(self, Kp=0., Ki=0., Kd=0., T=1.0):
        self.Kp = Kp  # P 제어기의 게인
        self.Ki = Ki
        self.Kd = Kd
        self.T = T
        self.e_k = np.zeros(3)
        self.u = 0

    def P(self, cur_error):
        self.e_k[0] = cur_error
        P = self.Kp * self.e_k[0] 
        return P
    #
    def PD(self, cur_error, tick=0.0):
        if tick > 0:
            self.T = tick
        self.e_k = np.roll(self.e_k, 1)
        self.e_k[0] = cur_error
        P = self.Kp * self.e_k[0]
        D = self.Kd / self.T * (self.e_k[0] - self.e_k[1])
        self.u = P + D # no delay
        return self.u

    def PID_control(self, cur_error):
        self.e_k = np.roll(self.e_k, 1)
        self.e_k[0] = cur_error
        P = self.Kp * (self.e_k[0] - self.e_k[1])
        I = self.Ki * self.T * self.e_k[0]
        D = self.Kd / self.T * (self.e_k[0] - 2 * self.e_k[1] + self.e_k[2])
        self.u += P + I + D  #u(k) +=  P + I + D
        return self.u
        # return self.Kp * self.e_k[0] 

    # def steering_angle_control(self, state, path_x, path_y, target_ind):
    #     # 목표 위치와 현재 위치 간의 각도 차이를 계산합니다.
    #     target_x = path_x[target_ind]
    #     target_y = path_y[target_ind]
    #     
    #     # 목표 지점과의 각도 오차를 계산
    #     dx = target_x - state.x
    #     dy = target_y - state.y
    #     angle_to_target = math.atan2(dy, dx)
    #     angle_error = angle_to_target - state.yaw
    #     
    #     # 각도 오차를 -pi에서 pi 사이로 정규화
    #     angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
    #
    #     # PD 제어를 통해 조향각(di)을 계산
    #     di = self.PD(angle_error)
    #     return di
