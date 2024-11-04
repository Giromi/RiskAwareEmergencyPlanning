from capdilib.convention import Waypoint, X, Y, YAW
from typing import List, Type
import matplotlib.pyplot as plt
import numpy as np
from utils.plot import plot_arrow
import math
import time 


class PathHanlder:

    '''
    @param path: list of Waypoint
    '''
    def __init__(self, path: List[Waypoint], PathPlanner: Type):
        self.path = path
        if len(path) < 2:
            raise ValueError("path should have more 2 waypoints")
        self.PathPlanner = PathPlanner



    def print_path(self):
        for i, wp in enumerate(self.path):
            print(f"Waypoint {i}: {wp[X]}, {wp[Y]}, {wp[YAW]}")
        print('path planner:', self.path_planner.__class__.__name__)
        print(self.path_planner.start)

    def calculate_path(self, selected_types: List[str] = None):
        self.path_x_list, self.path_y_list, self.path_yaw_list = [], [], []
        self.path_x, self.path_y, self.path_yaw = np.empty((0,)), np.empty((0,)), np.empty((0,))   # 크기가 0인 객체 배열, [], []
        for i in range(len(self.path) - 1):
            print(f"i: {i}")
            import time
            start = time.time()
            path_planner = self.PathPlanner(self.path[i], self.path[i + 1])
            cur_path_x, cur_path_y, cur_path_yaw, \
                cur_mode, cur_lengths = path_planner.calculate(selected_types)
            end = time.time()
            print(f"{end - start:.5f} sec")
             # NaN을 구분자로 삽입
            if i > 0:
                self.path_x = np.concatenate((self.path_x, [np.nan]))
                self.path_y = np.concatenate((self.path_y, [np.nan]))
                self.path_yaw = np.concatenate((self.path_yaw, [np.nan]))
            self.path_x = np.concatenate((self.path_x, cur_path_x), axis=0)
            self.path_y = np.concatenate((self.path_y, cur_path_y), axis=0)
            self.path_yaw = np.concatenate((self.path_yaw, cur_path_yaw), axis=0)

        # print(f'path_x: {self.path_x}, path_y: {self.path_y}, path_yaw: {self.path_yaw}')

        return self.path_x, self.path_y, self.path_yaw
            # print(f'path_x: {self.path_x}, path_y: {self.path_y}, path_yaw: {self.path_yaw}')
            # print(f"mode: {self.mode}, lengths: {self.lengths}")




