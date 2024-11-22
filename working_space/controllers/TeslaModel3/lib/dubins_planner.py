# 2024.11.17 Capdi2


from typing import List
from math import sin, cos, atan2, sqrt, acos, pi, hypot
from utils.operator import rot_mat_to_2d, angle_mod, mod2pi
import numpy as np

from lib.convention import X, Y, YAW, W, H
from lib.convention import Waypoint

""" Const Variables """

class DubinsPlanner:
    def __init__(self, start: Waypoint, end: Waypoint, 
                 curvature: float = 0.09 , step_size: float = 0.1):
        if len(start) != 3 or len(end) != 3:
            raise ValueError("start and end should be 3D points")
        self.start = start
        self.end = end
        self.curvature = curvature
        self.step_size = step_size
        self.local_goal = self.__find_local_goal()
        self.__PATH_TYPE_MAP = { 
            "LSL": self.__LSL,
            "RSR": self.__RSR, 
            "LSR": self.__LSR, 
            "RSL": self.__RSL, 
            "RLR": self.__RLR, 
            "LRL": self.__LRL, 
        }
        """ origin """
        self.alpha, self.beta, self.d = self.__find_from_origin()
        """ trigonometry """
        self.sin_a, self.sin_b, = sin(self.alpha), sin(self.beta)
        self.cos_a, self.cos_b  = cos(self.alpha), cos(self.beta)
        self.cos_ab = cos(self.alpha - self.beta)

    def __find_from_origin(self):
        dx = self.local_goal[X] - 0
        dy = self.local_goal[Y] - 0
        d = hypot(dx, dy) * self.curvature
        theta = mod2pi(atan2(dy, dx))
        alpha = mod2pi(-theta)
        beta = mod2pi(self.local_goal[YAW] - theta)
        print(f"dx: {dx}, dy: {dy}, d: {d}")
        print(f"theta: {theta}, alpha: {alpha}, beta: {beta}")
        return alpha, beta, d




    def __find_local_goal(self) -> list:
        l_rot = rot_mat_to_2d(self.start[YAW])
        le_xy = np.stack([self.end[X] - self.start[X], 
                          self.end[Y] - self.start[Y]]).T @ l_rot
        local_goal_x = le_xy[X]
        local_goal_y = le_xy[Y]
        local_goal_yaw = self.end[YAW] - self.start[YAW]

        return [local_goal_x, local_goal_y, local_goal_yaw]

    def calculate(self, selected_types: List[str] = None):
        self.local_goal = self.__find_local_goal()
        self.alpha, self.beta, self.d = self.__find_from_origin()
        planning_funcs = {
            key: val for key, val in self.__PATH_TYPE_MAP.items() 
            if selected_types is None or key in selected_types
        }
        '''
        None 이면 전체 선택
        selected_types 가 있으면 해당 타입만 선택
        '''

        # calculate local goal x, y, yaw
        lp_x_list, lp_y_list, lp_yaw_list, modes, lengths = \
                                self.__path_planning_from_origin( planning_funcs)
        # Convert a local coordinate path to the global coordinate
        rot = rot_mat_to_2d(-self.start[YAW])
        converted_xy = np.stack([lp_x_list, lp_y_list]).T @ rot
        x_list = converted_xy[:, 0] + self.start[X]
        y_list = converted_xy[:, 1] + self.start[Y]
        yaw_list = angle_mod(np.array(lp_yaw_list) + self.start[YAW])
        return x_list, y_list, yaw_list, modes, lengths



    def __path_planning_from_origin(self, planning_funcs):

        best_cost = float("inf")
        b_d1, b_d2, b_d3, b_mode = None, None, None, None

        for mode, __func in planning_funcs.items():
            d1, d2, d3 = __func()
            if d1 is None:
                continue

            cost = (abs(d1) + abs(d2) + abs(d3))
            if best_cost > cost:  # Select minimum length one.
                b_d1, b_d2, b_d3, b_mode, best_cost = d1, d2, d3, mode, cost

        lengths = [b_d1, b_d2, b_d3]
        x_list, y_list, yaw_list = self.__generate_local_course(lengths, b_mode)
        lengths = [length / self.curvature for length in lengths if length is not None]
        return x_list, y_list, yaw_list, b_mode, lengths

    def __generate_local_course(self, lengths, modes):
        p_x, p_y, p_yaw = [0.0], [0.0], [0.0]

        for (mode, length) in zip(modes, lengths):
            if length == 0.0:
                continue

            # set origin state
            origin_x, origin_y, origin_yaw = p_x[-1], p_y[-1], p_yaw[-1]

            current_length = self.step_size
            while abs(current_length + self.step_size) <= abs(length):
                p_x, p_y, p_yaw = self.__interpolate(current_length, mode,
                                               origin_x, origin_y, origin_yaw,
                                               p_x, p_y, p_yaw)
                current_length += self.step_size

            p_x, p_y, p_yaw = self.__interpolate(length, mode, origin_x,
                                           origin_y, origin_yaw, p_x, p_y, p_yaw)

        return p_x, p_y, p_yaw

    def __interpolate(self, length, mode, origin_x, origin_y,
                     origin_yaw, path_x, path_y, path_yaw):
        if mode == "S":
            path_x.append(origin_x + length / self.curvature * cos(origin_yaw))
            path_y.append(origin_y + length / self.curvature * sin(origin_yaw))
            path_yaw.append(origin_yaw)
        else:  # curve
            ldx = sin(length) / self.curvature
            ldy = 0.0
            if mode == "L":  # left turn
                ldy = (1.0 - cos(length)) / self.curvature
            elif mode == "R":  # right turn
                ldy = (1.0 - cos(length)) / -self.curvature
            gdx = cos(-origin_yaw) * ldx + sin(-origin_yaw) * ldy
            gdy = -sin(-origin_yaw) * ldx + cos(-origin_yaw) * ldy
            path_x.append(origin_x + gdx)
            path_y.append(origin_y + gdy)

            if mode == "L":  # left turn
                path_yaw.append(origin_yaw + length)
            elif mode == "R":  # right turn
                path_yaw.append(origin_yaw - length)

        return path_x, path_y, path_yaw

    def __LSL(self):

        p_squared = 2 + self.d ** 2 - (2 * self.cos_ab) + (2 * self.d * (self.sin_a - self.sin_b))
        if p_squared < 0:  # invalid configuration
            return None, None, None
        tmp = atan2((self.cos_b - self.cos_a), self.d + self.sin_a - self.sin_b)
        d1 = mod2pi(-self.alpha + tmp)
        d2 = sqrt(p_squared)
        d3 = mod2pi(self.beta - tmp)
        return d1, d2, d3

    def __RSR(self):
        
        p_squared = 2 + self.d ** 2 - (2 * self.cos_ab) + (2 * self.d * (self.sin_b - self.sin_a))
        if p_squared < 0:
            return None, None, None
        tmp = atan2((self.cos_a - self.cos_b), self.d - self.sin_a + self.sin_b)
        d1 = mod2pi(self.alpha - tmp)
        d2 = sqrt(p_squared)
        d3 = mod2pi(-self.beta + tmp)
        return d1, d2, d3


    def __LSR(self):
        p_squared = -2 + (self.d ** 2) + (2 * self.cos_ab) + (2 * self.d * (self.sin_a + self.sin_b))
        if p_squared < 0:
            return None, None, None
        d1 = sqrt(p_squared)
        tmp = atan2((-self.cos_a - self.cos_b), (self.d + self.sin_a + self.sin_b)) - atan2(-2.0, d1)
        d2 = mod2pi(-self.alpha + tmp)
        d3 = mod2pi(-mod2pi(self.beta) + tmp)
        return d2, d1, d3

    def __RSL(self):
        p_squared = self.d ** 2 - 2 + (2 * self.cos_ab) - (2 * self.d * (self.sin_a + self.sin_b))
        if p_squared < 0:
            return None, None, None
        d1 = sqrt(p_squared)
        tmp = atan2((self.cos_a + self.cos_b), (self.d - self.sin_a - self.sin_b)) - atan2(2.0, d1)
        d2 = mod2pi(self.alpha - tmp)
        d3 = mod2pi(self.beta - tmp)
        return d2, d1, d3


    def __RLR(self):
        tmp = (6.0 - self.d ** 2 + 2.0 * self.cos_ab + 2.0 * self.d * (self.sin_a - self.sin_b)) / 8.0
        if abs(tmp) > 1.0:
            return None, None, None
        d2 = mod2pi(2 * pi - acos(tmp))
        d1 = mod2pi(self.alpha - atan2(self.cos_a - self.cos_b, self.d - self.sin_a + self.sin_b) + d2 / 2.0)
        d3 = mod2pi(self.alpha - self.beta - d1 + d2)
        return d1, d2, d3


    def __LRL(self):
        tmp = (6.0 - self.d ** 2 + 2.0 * self.cos_ab + 2.0 * self.d * (- self.sin_a + self.sin_b)) / 8.0
        if abs(tmp) > 1.0:
            return None, None, None
        d2 = mod2pi(2 * pi - acos(tmp))
        d1 = mod2pi(-self.alpha - atan2(self.cos_a - self.cos_b, self.d + self.sin_a - self.sin_b) + d2 / 2.0)
        d3 = mod2pi(mod2pi(self.beta) - self.alpha - d1 + mod2pi(d2))
        return d1, d2, d3



