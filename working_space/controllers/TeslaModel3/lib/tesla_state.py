import math
from util.operator import angle_mod, webots_orientation_to_yaw
import numpy as np
from lib.convention import *

class IdealState:
    def __init__(self, dt, x=0, y=0, yaw=0, v=0):
        self.dt = dt
        self.t = 0
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.plot_time = 0
        # self.history = History()

    def __str__(self):
        return f't : {self.t}\nx : {self.x}\ny : {self.y}\nyaw : {self.yaw}\nv : {self.v}'

    def update(self, acceletation, delta):
        # self.history.append(self.t, self)

        if delta >= MAX_STEER:
            delta = MAX_STEER
        elif delta <= -MAX_STEER:
            delta = -MAX_STEER

        self.x += self.v * math.cos(self.yaw) * self.dt # 0.008
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw = self.yaw + self.v / WB * math.tan(delta) * self.dt
        self.v += acceletation * self.dt

    def is_simulation_pending(self):
        return MAX_TIME > self.t

    # def cal_distance(self, point_x, point_y):
    #     dx = self.x - point_x
    #     dy = self.y - point_y
    #     distance_error = math.hypot(dx, dy)
    #     return distance_error
    #
    # def cal_direction(self, point_x, point_y):
    #     dx = self.x - point_x
    #     dy = self.y - point_y
    #     direction_error = angle_mod(math.atan2(dy, dx) - self.yaw) # -pi ~ pi
    #     return direction_error  # [rad] -pi ~ pi


class History:  # Singleton Pattern
    def __init__(self):
        self.t = []
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []

    def append(self, t, cur_state):
        self.t.append(t)
        self.x.append(cur_state.x)
        self.y.append(cur_state.y)
        self.yaw.append(cur_state.yaw)
        self.v.append(cur_state.v)

    def clear(self):
        self.t.clear()
        self.x.clear()
        self.y.clear()
        self.yaw.clear()
        self.v.clear()



class TeslaState(IdealState): 
    def __init__(self, driver, dt, def_name='TeslaModel3'):
        self.driver = driver
        self.car_node = driver.getFromDef(def_name)
        super().__init__(dt, x=None, y=None, yaw=None, v=None) #  << self.history
        self.update()

    def set_all(self):
        pos, ori, speed = self.get_all()
        self.set_state(pos, ori, speed)

    def update(self, delta=0): # Different Parameter for blocking Parent's Method
        # self.history.append(self.get_time(), self)

        self.x, self.y, _ = self.get_position()
        self.yaw = self.get_yaw()
        self.v = self.get_speed()

        self.set_speed(TARGET_SPEED * 3.6) # [km/h]
        self.set_steering_angle(-delta)

    def set_speed(self, speed):
        self.car_node.setCruisingSpeed(speed)

    def get_position(self):
        return self.car_node.getPosition()

    def get_orientation(self):
        return self.car_node.getOrientation()
    
    def get_yaw(self):
        angle = webots_orientation_to_yaw(self.get_orientation())
        return angle

    def get_pose(self): 
        return self.car_node.getPose()

    def get_time_step(self):
        return self.car_node.getBasicTimeStep()

    def set_steering_angle(self, delta):
        self.driver.setSteeringAngle(delta)

    def get_speed(self):
        velocity = np.array(self.car_node.getVelocity())
        speed = np.linalg.norm(velocity)
        return speed

    def get_speed_km_h(self): # 뭔가 잘 안맞음
        return self.driver.getCurrentSpeed()
    
    def set_speed(self, speed):
        return self.driver.setCruisingSpeed(speed)

    def get_time(self):
        return self.driver.getTime()

    def is_simulation_pending(self):
        return self.driver.step() != -1 and MAX_TIME > self.get_time()
