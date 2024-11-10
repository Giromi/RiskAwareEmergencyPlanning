import math
from utils.operation import angle_mod

# Parameters
k = 0.1  # look forward gain
T = 0.1  # [s] time tick
WB = 2.89  # [m] wheel base of vehicle

class State:
    def __init__(self, dt=0.1, v=0.0, x=0.0, y=0.0, yaw=0.0):
        self.dt = dt
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v # [m/s]

    def update(self, a, cur_input):
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw += self.v / WB * math.tan(cur_input) * self.dt
        # self.v += a * dt 

    def cal_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        distance_error = math.hypot(dx, dy)
        # distance_error = error if not math.isnan(distance_error) else 0.0
        return distance_error

    '''
    @ return: -pi ~ pi
    '''
    def cal_direction(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        direction_error = angle_mod(math.atan2(dy, dx) - self.yaw) # -pi ~ pi
        # direction_error = error if not math.isnan(direction_error) else 0.0
        return direction_error

class State_list:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)

