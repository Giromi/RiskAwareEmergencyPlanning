import numpy as np
from scipy import interpolate
from lib.convention import *


class Spline2dPlanner:
    def __init__(self, points_waypoint, kind='cubic'):
        x, y  = points_waypoint[:, X], points_waypoint[:, Y]
        self.points_waypoint = points_waypoint
        self.ds = np.hypot(np.diff(x), np.diff(y))
        self.s = self.__calculate_s()
        self.sx = interpolate.interp1d(self.s, x, kind=kind)
        self.sy = interpolate.interp1d(self.s, y, kind=kind)

    def calculate(self):
        path = np.empty((0, 3))  # Updated to handle (x, y, yaw)
        s_array = np.arange(0, self.s[-1], 0.1)
        i = 0
        print(s_array)
        for cur_s in s_array:
            cur_x, cur_y = self.__calculate_position(cur_s)
            cur_path = np.vstack((cur_x, cur_y, self.points_waypoint[i, YAW]))
            path = np.vstack([path, cur_path.T])
            if i < len(self.points_waypoint) and int(cur_x) == self.points_waypoint[i, X] \
                                             and int(cur_y) == self.points_waypoint[i, Y]:
                i += 1
        return path

    def __calculate_s(self):
        s = [0.0]
        s.extend(np.cumsum(self.ds))
        return s

    def __calculate_position(self, cur_s):
        return self.sx(cur_s), self.sy(cur_s)


