""" Constant """
import numpy as np

X, Y, YAW, V = 0, 1, 2, 3
W, H = 2, 3

#
WB = 2.875          # [m] wheel base of vehicle

######### Static Variables #########
NX = 4                              # [#] x = x, number
NU = 2                              # [#] a = [accel, steer]
HORIZON_T = 10                              # [#] horizon length

R = np.diag([0.01, 0.01])           # [-] input cost matrix
Rd = np.diag([0.01, 1.0])           # [-] input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])   # [-] state cost matrix
Qf = Q                              # [-] state final matrix
MAX_TIME = 500.0  # max simulation time
MAX_ITER = 10#3                        # [iter] Max iteration
DU_TH = 0.1                         # [] iteration fnish param
N_IND_SEARCH = 10                   # [th] Search indextic Variables

GOAL_DIS = 10.0  #                  # [m] goal distance
STOP_SPEED = 10 / 3.6               # [m/s] stop speed
TARGET_SPEED = 108 / 3.6            # [m/s] target speed
DL = 1.0  # course tick
N_IND_SEARCH = 20   # Search index number
MAX_ACCEL = 3.2  # maximum accel [m/ss]
MAX_STEER = np.deg2rad(35.0) #35.0 # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 261.0 / 3.6  # maximum speed [m/s]  # 261.0 km/h
MIN_SPEED = 0  # minimum speed [m/s]

""" Vehicle Parameters for plot_car() """
LENGTH = 4.724  # [m]
WIDTH = 1.933  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]  # 20 inch
WHEEL_WIDTH = 0.2  # [m]
TREAD = 1.584  # [m]
OFFSET = 25
PLOT_CAR_TICK = 0.3

RRT_MAX_ITER = 100000

MSG_HEADER = "[Moral Machine]"
