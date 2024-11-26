from typing import List, Type
import numpy as np
import math
import time 

class PathHanlder:

    def __init__(self, waypoints: np.ndarray, PathPlanner: Type):
        self.waypoints = waypoints
        if len(waypoints) < 2:
            raise ValueError("path should have more 2 waypoints")
        self.PathPlanner = PathPlanner

    def calculate(self, selected_types: list[str] = None):
        path = np.empty((0, 3))  
        for i in range(len(self.waypoints) - 1):
            # print(f"i: {i}")
            path_planner = self.PathPlanner(self.waypoints[i], self.waypoints[i + 1])
            cur_path_x, cur_path_y, cur_path_yaw, cur_mode, cur_lengths \
                                            = path_planner.plan(selected_types)
            cur_path = np.vstack((cur_path_x, cur_path_y, cur_path_yaw))
            path = np.vstack([path, cur_path.T])
        return path


    # def print_path(self):
    #     for i, wp in enumerate(self.path):
    #         print(f"Waypoint {i}: {wp[X]}, {wp[Y]}, {wp[YAW]}")
    #     print('path planner:', self.path_planner.__class__.__name__)
    #     print(self.path_planner.start)
