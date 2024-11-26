# 2024.11.17 Capdi
"""
Path tracking simulation with iterative linear model predictive control for speed and steer control
author: Atsushi Sakai (@Atsushi_twi)
"""

from debug import *
import matplotlib.pyplot as plt
import cvxpy
import math
import numpy as np
from util.operator import angle_mod
from lib.tesla_state import IdealState, TeslaState
from lib.convention import *
from util.plot import plot_interval

class MPCTracker:

    def __init__(self, points_path, dt):
        self.points_path = points_path
        self.dt = dt

    def do_simulation(self, ideal_state): # NOT webots
        goal = np.array([self.points_path[-1, X], self.points_path[-1, Y]])
        # ideal_state.update()

        # initial yaw compensation
        if ideal_state.yaw - self.points_path[0, YAW] >= math.pi:
            ideal_state.yaw -= math.pi * 2.0
        elif ideal_state.yaw - self.points_path[0, YAW] <= -math.pi:
            ideal_state.yaw += math.pi * 2.0

        target_index, _ = self.calculate_nearest_index(ideal_state, self.points_path[:, X], self.points_path[:, Y], self.points_path[:, YAW], 0)
        odelta, oaccer = None, None

        cx = self.points_path[:, X]
        cy = self.points_path[:, Y]
        cyaw = self.smooth_yaw(self.points_path[:, YAW])
        sp = self.calculate_speed_profile(cx, cy, cyaw, TARGET_SPEED)
        dl = 1.0       # course tick

        while ideal_state.is_simulation_pending():
            x_ref, target_index, d_ref = self.calculate_ref_trajectory(ideal_state, cx, cy, cyaw, sp, dl, target_index)
            x_cur = [ideal_state.x, ideal_state.y , ideal_state.v, ideal_state.yaw]

            oaccer, odelta, ox, oy, oyaw, ov = self.iterative_linear_control(
                x_ref, x_cur, d_ref, oaccer, odelta)

            cur_delta, cur_accer = 0.0, 0.0
            if odelta is not None:
                cur_delta, cur_accer = odelta[0], oaccer[0]
                ideal_state.update(cur_accer, cur_delta) 
            ideal_state.t += self.dt

            if self.check_goal(ideal_state, goal, target_index, len(cx)):
                print("Goal")
                break
            plot_interval(ideal_state, cur_delta)
    ###### Do simulation END ######
    

    def track(self, tesla_state):
        print('Traking Start')
        goal = np.array([self.points_path[-1, X], self.points_path[-1, Y]])
        # tesla_state.update()  # 밖에서 함

        # initial yaw compensation
        # if tesla_state.yaw - self.points_path[0, YAW] >= math.pi:
        #     tesla_state.yaw -= math.pi * 2.0
        # elif tesla_state.yaw - self.points_path[0, YAW] <= -math.pi:
        #     tesla_state.yaw += math.pi * 2.0

        target_index, _ = self.calculate_nearest_index(tesla_state, self.points_path[:, X], self.points_path[:, Y], self.points_path[:, YAW], 0)
        odelta, oaccer = None, None

        cx = self.points_path[:, X]
        cy = self.points_path[:, Y]
        cyaw = self.smooth_yaw(self.points_path[:, YAW])
        sp = self.calculate_speed_profile(cx, cy, cyaw, TARGET_SPEED)
        dl = 1.0       # course tick

        while tesla_state.is_simulation_pending():

            x_ref, target_index, d_ref = self.calculate_ref_trajectory(tesla_state, cx, cy, cyaw, sp, dl, target_index)
            x_cur = [tesla_state.x, tesla_state.y , tesla_state.v, tesla_state.yaw]

            oaccer, odelta, ox, oy, oyaw, ov = self.iterative_linear_control(
                x_ref, x_cur, d_ref, oaccer, odelta)
            cur_delta, cur_accer = 0.0, 0.0
            if odelta is not None:
                cur_delta, cur_accer = odelta[0], oaccer[0]
                tesla_state.update(cur_delta) 
                # tesla_state.update(cur_delta)

            if self.check_goal(tesla_state, goal, target_index, len(cx)):
                print("Goal")
                break
            plot_interval(tesla_state, cur_delta)

    

    def calculate_ref_trajectory(self, state, cx, cy, cyaw, sp, dl, pind):
        x_ref = np.zeros((NX, HORIZON_T + 1))
        dref = np.zeros((1, HORIZON_T + 1))
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
        # print('dl:', dl)
        # print('travel:', travel)

        for i in range(HORIZON_T + 1):
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

        x = cvxpy.Variable((NX, HORIZON_T + 1))
        u = cvxpy.Variable((NU, HORIZON_T))


        cost = 0.0
        constraints = []

        for t in range(HORIZON_T):
            cost += cvxpy.quad_form(u[:, t], R)

            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

            A, B, C = self.get_linear_model_matrix(
                xbar[2, t], xbar[3, t], dref[0, t])
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

            if t < (HORIZON_T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= MAX_DSTEER * self.dt]

        cost += cvxpy.quad_form(xref[:, HORIZON_T] - x[:, HORIZON_T], Qf)

        constraints += [x[:, 0] == x0]
        constraints += [x[2, :] <= MAX_SPEED]
        constraints += [x[2, :] >= MIN_SPEED]
        constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
        constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]
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

        A = np.zeros((NX, NX))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.dt * math.cos(phi)
        A[0, 3] = - self.dt * v * math.sin(phi)
        A[1, 2] = self.dt * math.sin(phi)
        A[1, 3] = self.dt * v * math.cos(phi)
        A[3, 2] = self.dt * math.tan(delta) / WB

        B = np.zeros((NX, NU))
        B[2, 0] = self.dt
        B[3, 1] = self.dt * v / (WB * math.cos(delta) ** 2)

        C = np.zeros(NX)
        C[0] = self.dt * v * math.sin(phi) * phi
        C[1] = - self.dt * v * math.cos(phi) * phi
        C[3] = - self.dt * v * delta / (WB * math.cos(delta) ** 2)

        return A, B, C

    def predict_motion(self, x_cur, oa, od, x_ref):
        xbar = x_ref * 0.0
        for i, _ in enumerate(x_cur):
            xbar[i, 0] = x_cur[i]

        state = IdealState(self.dt, x=x_cur[X], y=x_cur[Y], yaw=x_cur[3], v=x_cur[2])
        for (ai, di, i) in zip(oa, od, range(1, HORIZON_T + 1)):
            state.update(ai, di)
            xbar[0, i] = state.x
            xbar[1, i] = state.y
            xbar[2, i] = state.v
            xbar[3, i] = state.yaw

        return xbar


    def iterative_linear_control(self, x_ref, x_cur, dref, oa, od):
        ox, oy, oyaw, ov = None, None, None, None

        if oa is None or od is None:
            oa = np.zeros(HORIZON_T)  # 크기 T의 0 배열
            od = np.zeros(HORIZON_T)


        for i in range(MAX_ITER):
            xbar = self.predict_motion(x_cur, oa, od, x_ref)
            poa, pod = oa[:], od[:]
            oa, od, ox, oy, oyaw, ov = self.linear_control(x_ref, xbar, x_cur, dref)

            # print(f"Updated oa: {oa}, od: {od}")

            du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
            if du <= DU_TH:
                break
        else:
            print("Iterative is max iter")

        return oa, od, ox, oy, oyaw, ov

    def check_goal(self, state, goal, tind, nind):

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        d = math.hypot(dx, dy)

        isgoal = (d <= GOAL_DIS)

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

        dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
        dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

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
    
    def smooth_yaw(self, yaw):

        for i in range(len(yaw) - 1):
            dyaw = yaw[i + 1] - yaw[i]

            while dyaw >= math.pi / 2.0:
                yaw[i + 1] -= math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

            while dyaw <= -math.pi / 2.0:
                yaw[i + 1] += math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

        return yaw







