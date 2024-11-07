"""
Nonlinear MPC simulation with CGMRES - author Atsushi Sakai (@Atsushi_twi)

Ref: Shunichi09/nonlinear_control: Implementing the nonlinear model predictive
    control, sliding mode control https://github.com/Shunichi09/nonlinear_control
"""
from math import cos, sin, radians, atan2
import matplotlib.pyplot as plt
import numpy as np

U_A_MAX = 1.0                 # 최대 가속도 (단위 확인) TODO (Webots에서 사용되는 차량의 실제 최대 가속도에 맞게 설정)
U_OMEGA_MAX = radians(45.0)   # 최대 각속도 [rad/s] TODO
PHI_V = 0.01                  # 속도에 대한 비용 함수의 파라미터 TODO   (페널티 값을 조정하여 제어기의 행동을 수정할 수 있다. 차량의 거동 특성에 맞춰 정밀한 제어를 위해 수정이 필요)
PHI_OMEGA = 0.01              # 각속도에 대한 비용 함수의 파라미터 TODO
WB = 0.289                     # wheel base [m] TODO

show_animation = True         # 애니메이션 표시 여부 설정


def differential_model(v, yaw, u_1, u_2):   # 차량의 움직임을 설명하는 미분 방정식 (현재속도, 현재방향, 가속도, 조향값)
    dx = cos(yaw) * v           # 속도와 방향(yaw)을 통해 x축 이동량 계산
    dy = sin(yaw) * v           # 속도와 방향(yaw)을 통해 y축 이동량 계산
    dv = u_1                    # 가속도 입력이 속도의 변화량
    # tangent is not good for nonlinear optimization
    d_yaw = v / WB * sin(u_2)   # 차량의 회전 변화율 계산

    return dx, dy, d_yaw, dv    # x, y, 방향(yaw), 속도의 변화율 반환


class TwoWheeledSystem:   # 두 바퀴로 움직이는 차량의 상태를 정의하고 업데이트

    def __init__(self, init_x, init_y, init_yaw, init_v):   # 정의 및 저장
        self.x = init_x       # 초기 x 위치
        self.y = init_y       # 초기 y 위치
        self.yaw = init_yaw   # 초기 방향 (라디안 단위)
        self.v = init_v       # 초기 속도
        self.history_x = [init_x]       # x 위치 히스토리 (리스트)
        self.history_y = [init_y]       # y 위치 히스토리
        self.history_yaw = [init_yaw]   # 방향 히스토리
        self.history_v = [init_v]       # 속도 히스토리

    def update_state(self, u_1, u_2, dt=0.01):   # 상태 업데이트  TODO (독립설정 필요 없을 듯..)
        dx, dy, d_yaw, dv = differential_model(self.v, self.yaw, u_1, u_2)   # differential_model 함수로부터 현재 속도와 방향을 바탕으로 변화율 계산

        # 시간 dt를 곱해 다음 정보 업데이트
        self.x += dt * dx   
        self.y += dt * dy   
        self.yaw += dt * d_yaw   
        self.v += dt * dv

        # save
        self.history_x.append(self.x)
        self.history_y.append(self.y)
        self.history_yaw.append(self.yaw)
        self.history_v.append(self.v)

class NMPCSimulatorSystem:   # 예측 상태 및 대응 상태(수반 상태)를 계산

    def calc_predict_and_adjoint_state(self, x, y, yaw, v, u_1s, u_2s, N, dt):
        # 상태 방정식을 사용하여 예측 상태를 계산
        x_s, y_s, yaw_s, v_s = self._calc_predict_states(x, y, yaw, v, u_1s, u_2s, N, dt)
        # 수반 방정식을 사용하여 대응 상태 계산
        lam_1s, lam_2s, lam_3s, lam_4s = self._calc_adjoint_states(x_s, y_s, yaw_s, v_s, u_2s, N, dt)

        return x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s

    def _calc_predict_states(self, x, y, yaw, v, u_1s, u_2s, N, dt):   # 차량의 상태를 N step 동안 예측하여 리스트에 저장
        x_s = [x]
        y_s = [y]
        yaw_s = [yaw]
        v_s = [v]

        for i in range(N):
            temp_x_1, temp_x_2, temp_x_3, temp_x_4 = self._predict_state_with_oylar(
                x_s[i], y_s[i], yaw_s[i], v_s[i], u_1s[i], u_2s[i], dt)
            x_s.append(temp_x_1)
            y_s.append(temp_x_2)
            yaw_s.append(temp_x_3)
            v_s.append(temp_x_4)

        return x_s, y_s, yaw_s, v_s

    def _calc_adjoint_states(self, x_s, y_s, yaw_s, v_s, u_2s, N, dt):  # 예측 상태와 반대로, 마지막 상태에서 역방향으로 수반 상태를 계산
        lam_x = [x_s[-1]]
        lam_y = [y_s[-1]]
        lam_yaw = [yaw_s[-1]]
        lam_v = [v_s[-1]]

        # 역방향 수반 상태 계산
        for i in range(N - 1, 0, -1):
            temp_lam_1, temp_lam_2, temp_lam_3, temp_lam_4 = self._adjoint_state_with_oylar(
                yaw_s[i], v_s[i], lam_x[0], lam_y[0], lam_yaw[0], lam_v[0],
                u_2s[i], dt)
            lam_x.insert(0, temp_lam_1)
            lam_y.insert(0, temp_lam_2)
            lam_yaw.insert(0, temp_lam_3)
            lam_v.insert(0, temp_lam_4)

        return lam_x, lam_y, lam_yaw, lam_v

    @staticmethod
    def _predict_state_with_oylar(x, y, yaw, v, u_1, u_2, dt):   # 오일러 방법을 이용하여 다음 상태를 계산

        dx, dy, dyaw, dv = differential_model(v, yaw, u_1, u_2)

        next_x = x + dt * dx         # 다음 x 위치
        next_y = y + dt * dy         # 다음 y 위치
        next_yaw = yaw + dt * dyaw   # 다음 방향(yaw)
        next_v = v + dt * dv         # 다음 속도

        return next_x, next_y, next_yaw, next_v

    @staticmethod
    def _adjoint_state_with_oylar(yaw, v, lam_1, lam_2, lam_3, lam_4, u_2, dt):

        # ∂H/∂x
        pre_lam_1 = lam_1 + dt * 0.0
        pre_lam_2 = lam_2 + dt * 0.0
        tmp1 = - lam_1 * sin(yaw) * v + lam_2 * cos(yaw) * v
        pre_lam_3 = lam_3 + dt * tmp1
        tmp2 = lam_1 * cos(yaw) + lam_2 * sin(yaw) + lam_3 * sin(u_2) / WB
        pre_lam_4 = lam_4 + dt * tmp2

        return pre_lam_1, pre_lam_2, pre_lam_3, pre_lam_4


class NMPCControllerCGMRES:
    """
    Attributes
    ------------
    zeta : float
        gain of optimal answer stability
    ht : float
        update value of NMPC this should be decided by zeta
    tf : float
        predict time
    alpha : float
        gain of predict time
    N : int
        predict step, discrete value
    threshold : float
        cgmres's threshold value
    input_num : int
        system input length, this should include dummy u and constraint variables
    max_iteration : int
        decide by the solved matrix size
    simulator : NMPCSimulatorSystem class
    u_1s : list of float
        estimated optimal system input
    u_2s : list of float
        estimated optimal system input
    dummy_u_1s : list of float
        estimated dummy input
    dummy_u_2s : list of float
        estimated dummy input
    raw_1s : list of float
        estimated constraint variable
    raw_2s : list of float
        estimated constraint variable
    history_u_1 : list of float
        time history of actual system input
    history_u_2 : list of float
        time history of actual system input
    history_dummy_u_1 : list of float
        time history of actual dummy u_1
    history_dummy_u_2 : list of float
        time history of actual dummy u_2
    history_raw_1 : list of float
        time history of actual raw_1
    history_raw_2 : list of float
        time history of actual raw_2
    history_f : list of float
        time history of error of optimal
    """

    def __init__(self):
        # parameters
        self.zeta = 100.  # stability gain TODO (너무 크면 응답이 느려지고, 너무 작으면 불안정할 수 있다)
        self.ht = 0.01    # 미분 근사 간격 TODO  (Webots의 시뮬레이션 주기와 유사하게 설정)
        self.tf = 3.0     # 예측 시간 TODO       (Webots에서 차량의 반응 속도와 목표에 맞게 조정)
        self.alpha = 0.5  # 예측 시간 게인 TODO  (예측 시간의 기하학적 증가 비율을 결정하는 게인. 차량의 민첩성과 제어기의 반응성을 고려하여 조정)
        self.N = 10       # 예측 단계 개수 TODO  (예측 단계 수를 늘리면 더 정밀한 예측이 가능하지만 계산 부하가 증가. Webots의 성능과 시뮬레이션 주기를 고려하여 적절하게 설정)
        self.threshold = 0.001   # CGMRES의 허용 오차 TODO  (최적화 계산의 종료 조건. 정밀한 제어를 위해 작은 값을 설정하지만, 계산 시간이 증가할 수 있다)
        self.input_num = 6       # 입력 변수 수(dummy 및 제약 변수 포함), constraints TODO (입력 변수 수를 조정할 필요는 없지만, 예측 단계 수(N)와의 곱을 고려해 최대 반복 횟수(max_iteration)를 설정)
        self.max_iteration = self.input_num * self.N

        # simulator
        self.simulator = NMPCSimulatorSystem()

        # initial input, initialize as 1.0
        self.u_1s = np.ones(self.N)
        self.u_2s = np.ones(self.N)
        self.dummy_u_1s = np.ones(self.N)
        self.dummy_u_2s = np.ones(self.N)
        self.raw_1s = np.zeros(self.N)
        self.raw_2s = np.zeros(self.N)

        self.history_u_1 = []
        self.history_u_2 = []
        self.history_dummy_u_1 = []
        self.history_dummy_u_2 = []
        self.history_raw_1 = []
        self.history_raw_2 = []
        self.history_f = []

    def calc_input(self, x, y, yaw, v, time):

        # calculating sampling time
        dt = self.tf * (1. - np.exp(-self.alpha * time)) / float(self.N)

        # x_dot
        x_1_dot, x_2_dot, x_3_dot, x_4_dot = differential_model(
            v, yaw, self.u_1s[0], self.u_2s[0])

        dx_1 = x_1_dot * self.ht
        dx_2 = x_2_dot * self.ht
        dx_3 = x_3_dot * self.ht
        dx_4 = x_4_dot * self.ht

        x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s = self.simulator.calc_predict_and_adjoint_state(
            x + dx_1, y + dx_2, yaw + dx_3, v + dx_4, self.u_1s, self.u_2s,
            self.N, dt)

        # Fxt:F(U,x+hx˙,t+h)
        Fxt = self._calc_f(v_s, lam_3s, lam_4s,
                           self.u_1s, self.u_2s, self.dummy_u_1s,
                           self.dummy_u_2s,
                           self.raw_1s, self.raw_2s, self.N)

        # F:F(U,x,t)
        x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s = self.simulator.calc_predict_and_adjoint_state(
            x, y, yaw, v, self.u_1s, self.u_2s, self.N, dt)

        F = self._calc_f(v_s, lam_3s, lam_4s,
                         self.u_1s, self.u_2s, self.dummy_u_1s, self.dummy_u_2s,
                         self.raw_1s, self.raw_2s, self.N)

        right = -self.zeta * F - ((Fxt - F) / self.ht)

        du_1 = self.u_1s * self.ht
        du_2 = self.u_2s * self.ht
        ddummy_u_1 = self.dummy_u_1s * self.ht
        ddummy_u_2 = self.dummy_u_2s * self.ht
        draw_1 = self.raw_1s * self.ht
        draw_2 = self.raw_2s * self.ht

        x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s = self.simulator.calc_predict_and_adjoint_state(
            x + dx_1, y + dx_2, yaw + dx_3, v + dx_4, self.u_1s + du_1,
            self.u_2s + du_2, self.N, dt)

        # Fuxt:F(U+hdU(0),x+hx˙,t+h)
        Fuxt = self._calc_f(v_s, lam_3s, lam_4s,
                            self.u_1s + du_1, self.u_2s + du_2,
                            self.dummy_u_1s + ddummy_u_1,
                            self.dummy_u_2s + ddummy_u_2,
                            self.raw_1s + draw_1, self.raw_2s + draw_2, self.N)

        left = ((Fuxt - Fxt) / self.ht)

        # calculating cgmres
        r0 = right - left
        r0_norm = np.linalg.norm(r0)

        vs = np.zeros((self.max_iteration, self.max_iteration + 1))
        vs[:, 0] = r0 / r0_norm

        hs = np.zeros((self.max_iteration + 1, self.max_iteration + 1))

        # in this case the state is 3(u and dummy_u)
        e = np.zeros((self.max_iteration + 1, 1))
        e[0] = 1.0

        ys_pre = None

        du_1_new, du_2_new, draw_1_new, draw_2_new = None, None, None, None
        ddummy_u_1_new, ddummy_u_2_new = None, None

        for i in range(self.max_iteration):
            du_1 = vs[::self.input_num, i] * self.ht
            du_2 = vs[1::self.input_num, i] * self.ht
            ddummy_u_1 = vs[2::self.input_num, i] * self.ht
            ddummy_u_2 = vs[3::self.input_num, i] * self.ht
            draw_1 = vs[4::self.input_num, i] * self.ht
            draw_2 = vs[5::self.input_num, i] * self.ht

            x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s = self.simulator.calc_predict_and_adjoint_state(
                x + dx_1, y + dx_2, yaw + dx_3, v + dx_4, self.u_1s + du_1,
                self.u_2s + du_2, self.N, dt)

            Fuxt = self._calc_f(v_s, lam_3s, lam_4s,
                                self.u_1s + du_1, self.u_2s + du_2,
                                self.dummy_u_1s + ddummy_u_1,
                                self.dummy_u_2s + ddummy_u_2,
                                self.raw_1s + draw_1, self.raw_2s + draw_2,
                                self.N)

            Av = ((Fuxt - Fxt) / self.ht)

            sum_Av = np.zeros(self.max_iteration)

            # Gram–Schmidt orthonormalization
            for j in range(i + 1):
                hs[j, i] = np.dot(Av, vs[:, j])
                sum_Av = sum_Av + hs[j, i] * vs[:, j]

            v_est = Av - sum_Av

            hs[i + 1, i] = np.linalg.norm(v_est)

            vs[:, i + 1] = v_est / hs[i + 1, i]

            inv_hs = np.linalg.pinv(hs[:i + 1, :i])
            ys = np.dot(inv_hs, r0_norm * e[:i + 1])

            judge_value = r0_norm * e[:i + 1] - np.dot(hs[:i + 1, :i], ys[:i])

            flag1 = np.linalg.norm(judge_value) < self.threshold

            flag2 = i == self.max_iteration - 1
            if flag1 or flag2:
                update_val = np.dot(vs[:, :i - 1], ys_pre[:i - 1]).flatten()
                du_1_new = du_1 + update_val[::self.input_num]
                du_2_new = du_2 + update_val[1::self.input_num]
                ddummy_u_1_new = ddummy_u_1 + update_val[2::self.input_num]
                ddummy_u_2_new = ddummy_u_2 + update_val[3::self.input_num]
                draw_1_new = draw_1 + update_val[4::self.input_num]
                draw_2_new = draw_2 + update_val[5::self.input_num]
                break

            ys_pre = ys

        # update input
        self.u_1s += du_1_new * self.ht
        self.u_2s += du_2_new * self.ht
        self.dummy_u_1s += ddummy_u_1_new * self.ht
        self.dummy_u_2s += ddummy_u_2_new * self.ht
        self.raw_1s += draw_1_new * self.ht
        self.raw_2s += draw_2_new * self.ht

        x_s, y_s, yaw_s, v_s, lam_1s, lam_2s, lam_3s, lam_4s = self.simulator.calc_predict_and_adjoint_state(
            x, y, yaw, v, self.u_1s, self.u_2s, self.N, dt)

        F = self._calc_f(v_s, lam_3s, lam_4s,
                         self.u_1s, self.u_2s, self.dummy_u_1s, self.dummy_u_2s,
                         self.raw_1s, self.raw_2s, self.N)

        print("norm(F) = {0}".format(np.linalg.norm(F)))

        # for save
        self.history_f.append(np.linalg.norm(F))
        self.history_u_1.append(self.u_1s[0])
        self.history_u_2.append(self.u_2s[0])
        self.history_dummy_u_1.append(self.dummy_u_1s[0])
        self.history_dummy_u_2.append(self.dummy_u_2s[0])
        self.history_raw_1.append(self.raw_1s[0])
        self.history_raw_2.append(self.raw_2s[0])

        return self.u_1s, self.u_2s

    @staticmethod
    def _calc_f(v_s, lam_3s, lam_4s, u_1s, u_2s, dummy_u_1s, dummy_u_2s,
                raw_1s, raw_2s, N):

        F = []
        for i in range(N):
            # ∂H/∂u(xi, ui, λi)
            F.append(u_1s[i] + lam_4s[i] + 2.0 * raw_1s[i] * u_1s[i])
            F.append(u_2s[i] + lam_3s[i] * v_s[i] /
                     WB * cos(u_2s[i]) ** 2 + 2.0 * raw_2s[i] * u_2s[i])
            F.append(-PHI_V + 2.0 * raw_1s[i] * dummy_u_1s[i])
            F.append(-PHI_OMEGA + 2.0 * raw_2s[i] * dummy_u_2s[i])

            # C(xi, ui, λi)
            F.append(u_1s[i] ** 2 + dummy_u_1s[i] ** 2 - U_A_MAX ** 2)
            F.append(u_2s[i] ** 2 + dummy_u_2s[i] ** 2 - U_OMEGA_MAX ** 2)

        return np.array(F)


def plot_figures(plant_system, controller, iteration_num,
                 dt):  # pragma: no cover
    # figure
    # time history
    fig_p = plt.figure()
    fig_u = plt.figure()
    fig_f = plt.figure()

    # trajectory
    fig_t = plt.figure()
    fig_trajectory = fig_t.add_subplot(111)
    fig_trajectory.set_aspect('equal')

    x_1_fig = fig_p.add_subplot(411)
    x_2_fig = fig_p.add_subplot(412)
    x_3_fig = fig_p.add_subplot(413)
    x_4_fig = fig_p.add_subplot(414)

    u_1_fig = fig_u.add_subplot(411)
    u_2_fig = fig_u.add_subplot(412)
    dummy_1_fig = fig_u.add_subplot(413)
    dummy_2_fig = fig_u.add_subplot(414)

    raw_1_fig = fig_f.add_subplot(311)
    raw_2_fig = fig_f.add_subplot(312)
    f_fig = fig_f.add_subplot(313)

    x_1_fig.plot(np.arange(iteration_num) * dt, plant_system.history_x)
    x_1_fig.set_xlabel("time [s]")
    x_1_fig.set_ylabel("x")

    x_2_fig.plot(np.arange(iteration_num) * dt, plant_system.history_y)
    x_2_fig.set_xlabel("time [s]")
    x_2_fig.set_ylabel("y")

    x_3_fig.plot(np.arange(iteration_num) * dt, plant_system.history_yaw)
    x_3_fig.set_xlabel("time [s]")
    x_3_fig.set_ylabel("yaw")

    x_4_fig.plot(np.arange(iteration_num) * dt, plant_system.history_v)
    x_4_fig.set_xlabel("time [s]")
    x_4_fig.set_ylabel("v")

    u_1_fig.plot(np.arange(iteration_num - 1) * dt, controller.history_u_1)
    u_1_fig.set_xlabel("time [s]")
    u_1_fig.set_ylabel("u_a")

    u_2_fig.plot(np.arange(iteration_num - 1) * dt, controller.history_u_2)
    u_2_fig.set_xlabel("time [s]")
    u_2_fig.set_ylabel("u_omega")

    dummy_1_fig.plot(np.arange(iteration_num - 1) *
                     dt, controller.history_dummy_u_1)
    dummy_1_fig.set_xlabel("time [s]")
    dummy_1_fig.set_ylabel("dummy u_1")

    dummy_2_fig.plot(np.arange(iteration_num - 1) *
                     dt, controller.history_dummy_u_2)
    dummy_2_fig.set_xlabel("time [s]")
    dummy_2_fig.set_ylabel("dummy u_2")

    raw_1_fig.plot(np.arange(iteration_num - 1) * dt, controller.history_raw_1)
    raw_1_fig.set_xlabel("time [s]")
    raw_1_fig.set_ylabel("raw_1")

    raw_2_fig.plot(np.arange(iteration_num - 1) * dt, controller.history_raw_2)
    raw_2_fig.set_xlabel("time [s]")
    raw_2_fig.set_ylabel("raw_2")

    f_fig.plot(np.arange(iteration_num - 1) * dt, controller.history_f)
    f_fig.set_xlabel("time [s]")
    f_fig.set_ylabel("optimal error")

    fig_trajectory.plot(plant_system.history_x,
                        plant_system.history_y, "-r")
    fig_trajectory.set_xlabel("x [m]")
    fig_trajectory.set_ylabel("y [m]")
    fig_trajectory.axis("equal")

    # start state
    plot_car(plant_system.history_x[0],
             plant_system.history_y[0],
             plant_system.history_yaw[0],
             controller.history_u_2[0],
             )

    # goal state
    plot_car(0.0, 0.0, 0.0, 0.0)

    plt.show()


def plot_car(x, y, yaw, steer=0.0, truck_color="-k"):  # pragma: no cover

    # Vehicle parameters
    LENGTH = 0.4          # 차량 길이 [m] TODO
    WIDTH = 0.2           # 차량 폭 [m] TODO
    BACK_TO_WHEEL = 0.1   # 뒷바퀴부터 차량 후면까지 거리 [m] TODO
    WHEEL_LEN = 0.03      # 바퀴 길이 [m] TODO
    WHEEL_WIDTH = 0.02    # 바퀴 폭 [m] TODO
    TREAD = 0.07          # 차량 바퀴 간 폭 [m] TODO
 
    outline = np.array(
        [[-BACK_TO_WHEEL, (LENGTH - BACK_TO_WHEEL), (LENGTH - BACK_TO_WHEEL),
          -BACK_TO_WHEEL, -BACK_TO_WHEEL],
         [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array(
        [[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH -
          TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[cos(yaw), sin(yaw)],
                     [-sin(yaw), cos(yaw)]])
    Rot2 = np.array([[cos(steer), sin(steer)],
                     [-sin(steer), cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truck_color)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truck_color)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truck_color)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truck_color)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truck_color)
    plt.plot(x, y, "*")


def animation(plant, controller, dt):
    skip = 2  # skip index for animation  TODO (큰 의미는 없음)

    for t in range(1, len(controller.history_u_1), skip):
        x = plant.history_x[t]
        y = plant.history_y[t]
        yaw = plant.history_yaw[t]
        v = plant.history_v[t]
        accel = controller.history_u_1[t]
        time = t * dt

        if abs(v) <= 0.01:
            steer = 0.0
        else:
            steer = atan2(controller.history_u_2[t] * WB / v, 1.0)

        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(plant.history_x, plant.history_y, "-r", label="trajectory")
        plot_car(x, y, yaw, steer=steer)
        plt.axis("equal")
        plt.grid(True)
        plt.title("Time[s]:" + str(round(time, 2)) +
                  ", accel[m/s]:" + str(round(accel, 2)) +
                  ", speed[km/h]:" + str(round(v * 3.6, 2)))
        plt.pause(0.0001)

    plt.close("all")


def main():
    # simulation time
    dt = 0.1  # 시뮬레이션 시간 간격 TODO (Webots의 기본 시간 간격에 맞게 설정)
    iteration_time = 150.0  # 총 시뮬레이션 시간 [s] TODO (Webots에서 수행할 시뮬레이션의 총 시간을 설정)

    init_x = -4.5  # 초기 x 위치 TODO Webots에서 ref
    init_y = -2.5  # 초기 y 위치 TODO 
    init_yaw = radians(45.0)  # 초기 방향 (라디안 단위) TODO 
    init_v = 100/3.6  # 초기 속도 TODO 

    # plant
    plant_system = TwoWheeledSystem(
        init_x, init_y, init_yaw, init_v)

    # controller
    controller = NMPCControllerCGMRES()

    iteration_num = int(iteration_time / dt)
    for i in range(1, iteration_num):
        time = float(i) * dt
        # make input
        u_1s, u_2s = controller.calc_input(
            plant_system.x, plant_system.y, plant_system.yaw, plant_system.v,
            time)
        # update state
        plant_system.update_state(u_1s[0], u_2s[0])

    if show_animation:  # pragma: no cover
        animation(plant_system, controller, dt)
        plot_figures(plant_system, controller, iteration_num, dt)


if __name__ == "__main__":
    main()
