# 2024.11.17 Capdi
"""
Path tracking simulation with iterative linear model predictive control for speed and steer control
author: Atsushi Sakai (@Atsushi_twi)
"""

from util.debug import *
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

    def do_simulation(self, driver, ideal_state): # NOT webots
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
        # sp = self.calculate_speed_profile(cx, cy, cyaw, TARGET_SPEED)
        dl = 1.0       # course tick

        while ideal_state.is_simulation_pending(driver):
            x_ref, target_index, d_ref = self.calculate_ref_trajectory(ideal_state,
                                                   cx, cy, cyaw, dl, target_index)
            x_cur = [ideal_state.x, ideal_state.y , ideal_state.v, ideal_state.yaw]

            oaccer, odelta, ox, oy, oyaw, ov = self.iterative_linear_control(
                x_ref, x_cur, d_ref, oaccer, odelta)

            cur_delta, cur_accer = 0.0, 0.0
            if odelta is not None:
                cur_delta, cur_accer = odelta[0], oaccer[0]
                ideal_state.update(cur_accer, cur_delta) 
            ideal_state.add_time(self.dt)

            print(f"ideal_state : {ideal_state.v * 3.6}")

            if self.check_goal(ideal_state, goal, target_index, len(cx)):
                print("Goal")
                break
            plot_interval(ideal_state, cur_delta, cx, cy, target_index)
    ###### Do simulation END ######
    

    def track(self, tesla_state):
        print('Traking Start')
        goal = np.array([self.points_path[-1, X], self.points_path[-1, Y]])

        # initial yaw compensation
        # if tesla_state.yaw - self.points_path[0, YAW] >= math.pi:
        #     tesla_state.yaw -= math.pi * 2.0
        # elif tesla_state.yaw - self.points_path[0, YAW] <= -math.pi:
        #     tesla_state.yaw += math.pi * 2.0

        target_index, _ = self.calculate_nearest_index(tesla_state, self.points_path[:, X], self.points_path[:, Y], self.points_path[:, YAW], 0)
        odelta, oaccel = None, None

        cx = self.points_path[:, X]
        cy = self.points_path[:, Y]
        cyaw = self.smooth_yaw(self.points_path[:, YAW])
        # sp = self.calculate_speed_profile(cx, cy, cyaw, TARGET_SPEED)
        dl = 1.0       # course tick

        while tesla_state.is_simulation_pending():
            tesla_state.update()
            x_ref, target_index, d_ref = self.calculate_ref_trajectory(
                                tesla_state, cx, cy, cyaw, dl, target_index)
            x_cur = [tesla_state.x, tesla_state.y, tesla_state.yaw]
            odelta, ox, oy, oyaw = self.iterative_linear_control(
                        x_ref, x_cur, d_ref, oaccel, odelta, tesla_state.v)
            cur_delta = 0.0
            if odelta is not None:
                cur_delta = odelta[0]
                tesla_state.set_steering_angle(cur_delta)

            print(f"Tesla State : {tesla_state.v * 3.6}")

            if self.check_goal(tesla_state, goal, target_index, len(cx)):
                print("Goal")
                break
            plot_interval(tesla_state, cur_delta, cx, cy, target_index)

    

    def calculate_ref_trajectory(self, state, cx, cy, cyaw, dl, pind):
        z_ref = np.zeros((NX, HORIZON_T + 1))
        d_ref = np.zeros((1, HORIZON_T + 1))
        ncourse = len(cx)
        ind, _ = self.calculate_nearest_index(state, cx, cy, cyaw, pind)

        if pind >= ind:
            ind = pind

        z_ref[0, 0] = cx[ind]
        z_ref[1, 0] = cy[ind]
        z_ref[2, 0] = cyaw[ind]
        d_ref[0, 0] = 0.0  # steer operational point should be 0

        travel = 0.0
        # print('dl:', dl)
        # print('travel:', travel)

        for i in range(HORIZON_T + 1):
            travel += abs(state.v) * self.dt
            dind = int(round(travel / dl))

            if (ind + dind) < ncourse:
                z_ref[0, i] = cx[ind + dind]
                z_ref[1, i] = cy[ind + dind]
                z_ref[2, i] = cyaw[ind + dind]
                d_ref[0, i] = 0.0
            else:
                z_ref[0, i] = cx[ncourse - 1]
                z_ref[1, i] = cy[ncourse - 1]
                z_ref[2, i] = cyaw[ncourse - 1]
                d_ref[0, i] = 0.0

        return z_ref, ind, d_ref



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

    def iterative_linear_control(self, z_ref, z_cur, d_ref, oa, od, cur_v):
        ox, oy, oyaw = None, None, None

        if od is None:
            od = np.zeros(HORIZON_T)

        for _ in range(MAX_ITER):
            z_bar = self.predict_motion(z_cur, oa, od, z_ref, cur_v)
            pod = od[:]
            od, ox, oy, oyaw = self.linear_control(z_ref, z_bar, z_cur, d_ref, cur_v)

            ################
            if oa is None or od is None:
                od = pod[:]
                print("Error: Cannot solve mpc..")
                break
            ################

            du = sum(abs(od - pod))  # calc u change value
            if du <= DU_TH:
                break
        else:
            print("Iterative is max iter")
        return od, ox, oy, oyaw


    def linear_control(self, z_ref, z_bar, x0, dref, cur_v):
        """
        linear mpc control
        z_ref: reference point
        z_bar: operational point
        x0: initial state
        dref: reference steer angle
        """

        z = cvxpy.Variable((NX, HORIZON_T + 1))
        u = cvxpy.Variable((NU, HORIZON_T))

        cost = 0.0
        constraints = []

        for t in range(HORIZON_T):
            cost += cvxpy.quad_form(u[:, t], R)

            if t != 0:
                cost += cvxpy.quad_form(z_ref[:, t] - z[:, t], Q)

            A, B, C = self.get_linear_model_matrix(cur_v, z_bar[2, t], dref[0, t])
            constraints += [ z[:, t + 1] == A @ z[:, t] + B @ u[:, t] + C ]

            if t < (HORIZON_T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
                constraints += [cvxpy.abs(u[0, t + 1] - u[0, t]) <= MAX_DSTEER * self.dt]

        cost += cvxpy.quad_form(z_ref[:, HORIZON_T] - z[:, HORIZON_T], Q)

        constraints += [z[:, 0] == x0]
        constraints += [cvxpy.abs(u[0, :]) <= MAX_STEER]
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.ECOS, verbose=False)

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ox = self.get_nparray_from_matrix(z.value[0, :])
            oy = self.get_nparray_from_matrix(z.value[1, :])
            oyaw = self.get_nparray_from_matrix(z.value[2, :])
            odelta = self.get_nparray_from_matrix(u.value[0, :])
        else:
            print("Error: Cannot solve mpc..")
            odelta, ox, oy, oyaw = None, None, None, None
        return odelta, ox, oy, oyaw


    def get_linear_model_matrix(self, cur_v, phi, delta):
        A = np.zeros((NX, NX))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[0, 2] = - self.dt * cur_v * math.sin(phi)
        A[1, 2] = self.dt * cur_v * math.cos(phi)


        B = np.zeros((NX, NU))
        B[2, 0] = self.dt * cur_v / (WB * math.cos(delta) ** 2)

        C = np.zeros(NX)
        C[0] = self.dt * cur_v * math.sin(phi) * phi
        C[1] = - self.dt * cur_v * math.cos(phi) * phi
        C[2] = - self.dt * cur_v * delta / (WB * math.cos(delta) ** 2)

        return A, B, C

    def predict_motion(self, x_cur, oa, od, x_ref, cur_v):
        z_bar = x_ref * 0.0
        for i, _ in enumerate(x_cur):
            z_bar[i, 0] = x_cur[i]

        state = IdealState(self.dt, x=x_cur[X], y=x_cur[Y], yaw=x_cur[YAW], v=cur_v)
        for (di, i) in zip(od, range(1, HORIZON_T + 1)):
            state.update(di)
            z_bar[0, i] = state.x
            z_bar[1, i] = state.y
            z_bar[2, i] = state.yaw
        return z_bar


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







