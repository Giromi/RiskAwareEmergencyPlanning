# 2024.11.17 Capdi
"""
Path tracking simulation with iterative linear model predictive control for speed and steer control
author: Atsushi Sakai (@Atsushi_twi)
"""
import matplotlib.pyplot as plt
import cvxpy
import math
import numpy as np
from utils.operator import angle_mod
from lib.tesla_state import IdealState


class MPCTracker:
    ######### Static Variables #########
    NX = 4                              # [#] x = x, number
    NU = 2                              # [#] a = [accel, steer]
    T = 5                               # [#] horizon length

    R = np.diag([0.01, 0.01])           # [-] input cost matrix
    Rd = np.diag([0.01, 1.0])           # [-] input difference cost matrix
    Q = np.diag([1.0, 1.0, 0.5, 0.5])   # [-] state cost matrix
    Qf = Q                              # [-] state final matrix

    MAX_ITER = 3                        # [iter] Max iteration
    DU_TH = 0.1                         # [] iteration finish param
    N_IND_SEARCH = 10                   # [th] Search indextic Variables

    GOAL_DIS = 20.0  #                  # [m] goal distance
    STOP_SPEED = 10 / 3.6               # [m/s] stop speed
    TARGET_SPEED = 108 / 3.6            # [m/s] target speed


    def __init__(self, dt):
        self.dt = dt

    def calculate_ref_trajectory(self, state, cx, cy, cyaw, sp, dl, pind):
        x_ref = np.zeros((MPCTracker.NX, MPCTracker.T + 1))
        dref = np.zeros((1, MPCTracker.T + 1))
        ncourse = len(cx)
        ind, _ = self.calculate_nearest_index(state, cx, cy, cyaw, pind)

        if pind >= ind:
            ind = pind

        x_ref[0, 0] = cx[ind]
        x_ref[1, 0] = cy[ind]
        x_ref[2, 0] = sp[ind]
        x_ref[3, 0] = cyaw[ind]
        dref[0, 0] = 0.0  # steer operational point should be 0

        travel = 0.0

        for i in range(MPCTracker.T + 1):
            travel += abs(state.v) * self.dt
            dind = int(round(travel / dl))

            if (ind + dind) < ncourse:
                x_ref[0, i] = cx[ind + dind]
                x_ref[1, i] = cy[ind + dind]
                x_ref[2, i] = sp[ind + dind]
                x_ref[3, i] = cyaw[ind + dind]
                dref[0, i] = 0.0
            else:
                x_ref[0, i] = cx[ncourse - 1]
                x_ref[1, i] = cy[ncourse - 1]
                x_ref[2, i] = sp[ncourse - 1]
                x_ref[3, i] = cyaw[ncourse - 1]
                dref[0, i] = 0.0
        print("x_ref : ", x_ref)

        return x_ref, ind, dref



    def calculate_speed_profile(self, cx, cy, cyaw, target_speed):
        speed_profile = [target_speed] * len(cx)
        direction = 1.0  # forward
        # Set stop point
        for i in range(len(cx) - 1):
            dx = cx[i + 1] - cx[i]
            dy = cy[i + 1] - cy[i]

            move_direction = math.atan2(dy, dx)

            if dx != 0.0 and dy != 0.0:
                dangle = abs(angle_mod(move_direction - cyaw[i]))
                if dangle >= math.pi / 4.0:
                    direction = -1.0
                else:
                    direction = 1.0

            if direction != 1.0:
                speed_profile[i] = - target_speed
            else:
                speed_profile[i] = target_speed

        speed_profile[-1] = 0.0
        
        return speed_profile

    def linear_control(self, xref, xbar, x0, dref):
        """
        linear mpc control

        xref: reference point
        xbar: operational point
        x0: initial state
        dref: reference steer angle
        """

        # print('xref :', xref.shape, xref)
        # print('xbar :', xbar.shape, xbar)
        # print('x0 :', x0)
        # print('dref :', dref.shape, dref)

        x = cvxpy.Variable((MPCTracker.NX, MPCTracker.T + 1))
        u = cvxpy.Variable((MPCTracker.NU, MPCTracker.T))


        cost = 0.0
        constraints = []

        for t in range(MPCTracker.T):
            cost += cvxpy.quad_form(u[:, t], MPCTracker.R)

            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], MPCTracker.Q)

            A, B, C = self.get_linear_model_matrix(
                xbar[2, t], xbar[3, t], dref[0, t])
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

            if t < (MPCTracker.T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], MPCTracker.Rd)
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= IdealState.MAX_DSTEER * self.dt]

        cost += cvxpy.quad_form(xref[:, MPCTracker.T] - x[:, MPCTracker.T], MPCTracker.Qf)

        constraints += [x[:, 0] == x0]
        constraints += [x[2, :] <= IdealState.MAX_SPEED]
        constraints += [x[2, :] >= IdealState.MIN_SPEED]
        constraints += [cvxpy.abs(u[0, :]) <= IdealState.MAX_ACCEL]
        constraints += [cvxpy.abs(u[1, :]) <= IdealState.MAX_STEER]
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.ECOS, verbose=False)

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ox = self.get_nparray_from_matrix(x.value[0, :])
            oy = self.get_nparray_from_matrix(x.value[1, :])
            ov = self.get_nparray_from_matrix(x.value[2, :])
            oyaw = self.get_nparray_from_matrix(x.value[3, :])
            oa = self.get_nparray_from_matrix(u.value[0, :])
            odelta = self.get_nparray_from_matrix(u.value[1, :])

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov


    def get_linear_model_matrix(self, v, phi, delta):

        A = np.zeros((MPCTracker.NX, MPCTracker.NX))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.dt * math.cos(phi)
        A[0, 3] = - self.dt * v * math.sin(phi)
        A[1, 2] = self.dt * math.sin(phi)
        A[1, 3] = self.dt * v * math.cos(phi)
        A[3, 2] = self.dt * math.tan(delta) / IdealState.WB

        B = np.zeros((MPCTracker.NX, MPCTracker.NU))
        B[2, 0] = self.dt
        B[3, 1] = self.dt * v / (IdealState.WB * math.cos(delta) ** 2)

        C = np.zeros(MPCTracker.NX)
        C[0] = self.dt * v * math.sin(phi) * phi
        C[1] = - self.dt * v * math.cos(phi) * phi
        C[3] = - self.dt * v * delta / (IdealState.WB * math.cos(delta) ** 2)

        return A, B, C

    def predict_motion(self, x_cur, oa, od, x_ref):
        xbar = x_ref * 0.0
        for i, _ in enumerate(x_cur):
            xbar[i, 0] = x_cur[i]

        state = IdealState(self.dt, x=x_cur[0], y=x_cur[1], yaw=x_cur[3], v=x_cur[2])
        for (ai, di, i) in zip(oa, od, range(1, MPCTracker.T + 1)):
            state.update(ai, di)
            xbar[0, i] = state.x
            xbar[1, i] = state.y
            xbar[2, i] = state.v
            xbar[3, i] = state.yaw

        return xbar


    def iterative_linear_control(self, x_ref, x_cur, dref, oa, od):
        ox, oy, oyaw, ov = None, None, None, None

        if oa is None or od is None:
            oa = np.zeros(MPCTracker.T)  # 크기 T의 0 배열
            od = np.zeros(MPCTracker.T)


        for i in range(MPCTracker.MAX_ITER):
            xbar = self.predict_motion(x_cur, oa, od, x_ref)
            poa, pod = oa[:], od[:]
            oa, od, ox, oy, oyaw, ov = self.linear_control(x_ref, xbar, x_cur, dref)

            # print(f"Updated oa: {oa}, od: {od}")

            du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
            if du <= MPCTracker.DU_TH:
                break
        else:
            print("Iterative is max iter")

        return oa, od, ox, oy, oyaw, ov

    def check_goal(self, state, goal, tind, nind):

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        d = math.hypot(dx, dy)

        print('d :', d, 'vs', MPCTracker.GOAL_DIS)

        isgoal = (d <= MPCTracker.GOAL_DIS)

        if abs(tind - nind) >= 5:
            isgoal = False

        # TODO : 최종적으로는 10km/h 이하로 속도가 떨어지면 종료
        # isstop = (abs(self.v) <= STOP_SPEED)

        if isgoal: #and isstop:
            return True
        return False

    def get_nparray_from_matrix(self, x):
        return np.array(x).flatten()

    def calculate_nearest_index(self, state, cx, cy, cyaw, pind):

        dx = [state.x - icx for icx in cx[pind:(pind + MPCTracker.N_IND_SEARCH)]]
        dy = [state.y - icy for icy in cy[pind:(pind + MPCTracker.N_IND_SEARCH)]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind) + pind

        mind = math.sqrt(mind)

        dxl = cx[ind] - state.x
        dyl = cy[ind] - state.y

        angle = angle_mod(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind
