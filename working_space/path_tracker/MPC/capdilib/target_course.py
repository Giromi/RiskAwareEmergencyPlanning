import numpy as np
import math
from utils.operation import angle_mod


class TargetCourse:
    def __init__(self, target_x, target_y):
        self.tx = target_x
        self.ty = target_y
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.x - xp for xp in self.tx]
            dy = [state.y - yp for yp in self.ty]
            print(f'57 state.x: {state.x} - self.tx: {self.tx[57]}') 
            print(f'57 state.y: {state.y} - self.ty: {self.ty[57]}')
            d = np.hypot(dx, dy) # 빗변 norm L2 []
            i = np.argmin(d)
            print('i :', i)
            print('d :', d[i])
            self.old_nearest_point_index = i
        else:
            i = self.old_nearest_point_index
            distance_this_index = state.cal_distance(self.tx[i], self.ty[i])
            while i < len(self.tx) - 1:
                distance_next_index = state.cal_distance(self.tx[i + 1], self.ty[i + 1])
                if distance_this_index < distance_next_index:
                    break
                i = i + 1 if (i + 1) < len(self.tx) else i
                distance_this_index = distance_next_index
            self.old_nearest_point_index = i

        return i


