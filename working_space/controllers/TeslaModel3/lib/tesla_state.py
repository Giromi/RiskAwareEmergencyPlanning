import math
from utils.operator import angle_mod, webots_orientation_to_yaw
import numpy as np

class IdealState:
    ######### Static Variables #########
    WB = 2.875          # [m] wheel base of vehicle
    N_IND_SEARCH = 10   # Search index number
    MAX_STEER = np.deg2rad(35.0)  # maximum steering angle [rad]
    MAX_SPEED = 261.0 / 3.6  # maximum speed [m/s]  # 261.0 km/h
    MIN_SPEED = 0  # minimum speed [m/s]
    MAX_ACCEL = 3.2  # maximum accel [m/ss]
    MAX_DSTEER = np.deg2rad(90.0)  # maximum steering speed [rad/s]
    # LENGTH = 4.724  # [m]
    # WIDTH = 1.933  # [m]
    # BACKTOWHEEL = 1.0  # [m]
    # WHEEL_LEN = 0.3  # [m]  # 20 inch
    # WHEEL_WIDTH = 0.2  # [m]
    # TREAD = 1.584  # [m]
    ####################################

    def __init__(self, dt, x=0, y=0, yaw=0, v=0):
        self.dt = dt
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def __str__(self):
        return f'x : {self.x}, y : {self.y}, yaw : {self.yaw}, v : {self.v}'

    # def set_state(self, pos, ori, speed):
    #     self.x = pos[0]                 # [m]
    #     self.y = pos[1]                 # [m]
    #     self.yaw = rot_mat_to_yaw(ori)  # [rad] 0 ~ 2pi
    #     self.speed = speed              # [m/s]

    def update(self, acceletation, delta):
        if delta >= IdealState.MAX_STEER:
            delta = IdealState.MAX_STEER
        elif delta <= -IdealState.MAX_STEER:
            delta = -IdealState.MAX_STEER

        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw = self.yaw + self.v / IdealState.WB * math.tan(delta) * self.dt
        # self._speed += acceletation * self.dt

    def cal_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        distance_error = math.hypot(dx, dy)
        # distance_error = error if not math.isnan(distance_error) else 0.0
        return distance_error

    def cal_direction(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        direction_error = angle_mod(math.atan2(dy, dx) - self.yaw) # -pi ~ pi
        # direction_error = error if not math.isnan(direction_error) else 0.0
        return direction_error  # [rad] -pi ~ pi




class TeslaState(IdealState):

    def __init__(self, driver, dt, def_name='TeslaModel3'):
        self.driver = driver
        self.car_node = driver.getFromDef(def_name)
        super().__init__(dt, x=None, y=None, yaw=None, v=None)
        self.update()

    def set_all(self):
        pos, ori, speed = self.get_all()
        self.set_state(pos, ori, speed)

    def update(self, delta=0): # Different Parameter for blocking Parent's Method
        print('update')
        self.x, self.y, _ = self.get_position()
        print('x, y :', self.x, self.y)
        self.yaw = self.get_yaw()
        self.v = self.get_speed()
        self.set_steering_angle(delta)


    def set_speed(self, speed):
        self.car_node.setCruisingSpeed(speed)

    def get_position(self):
        return self.car_node.getPosition()

    def get_orientation(self):
        return self.car_node.getOrientation()
    
    def get_yaw(self):
        angle = webots_orientation_to_yaw(self.get_orientation())
        print('angle :', angle)
        return angle

    def get_pose(self): 
        return self.car_node.getPose()

    def get_time_step(self):
        return self.car_node.getBasicTimeStep()

    def set_steering_angle(self, delta):
        self.driver.setSteeringAngle(delta)

    def get_speed(self):
        return self.driver.getCurrentSpeed()


    

class History:  # Singleton Pattern
    ######### Static Variables #########

    def __init__(self):
        self.x_list = []
        self.y_list = []
        self.yaw_list = []
        self.v_list = []
        self.t_list = []
    ####################################

    def append(self, t, cur_state):
        self.x_list.append(cur_state.x)
        self.y_list.append(cur_state.y)
        self.yaw_list.append(cur_state.yaw)
        self.v_list.append(cur_state.v)
        self.t_list.append(t)

    def clear(self):
        self.x_list.clear()
        self.y_list.clear()
        self.yaw_list.clear()
        self.v_list.clear()
        self.t_list.clear()
