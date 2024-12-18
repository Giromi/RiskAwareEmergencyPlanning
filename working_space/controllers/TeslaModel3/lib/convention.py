""" Constant """
import numpy as np

X, Y, YAW, V    = 0, 1, 2, 3
W, H            = 2, 3

#
""" MPC Parameters """
WB                  = 2.875          # [m] wheel base of vehicle
HORIZON_T           = 2                               # [#] horizon length(10) 오히려 너무 크면 이상해짐 속도를 모르기때문에
NU                  = 1                                 # [#] a = [accel, steer](2)
R                   = np.diag([0.10])             # [-] input cost matrix([0.01, 0.01])
Rd                  = np.diag([1.0])              # [-] input difference cost matrix([0.01, 1.0])
NX                  = 3                                 # [#] x = x, number(4)
Q                   = np.diag([1.0, 1.0, 1.50])    # [-] state cost matrix([1.0, 1.0, 0.5, 0.5])
MAX_TIME            = 500.0                             # max simulation time
MAX_ITER            = 3                                 # [iter] Max iteration
DU_TH               = 0.1                               # [] iteration finish param(0.1)
N_IND_SEARCH        = 10                                # [th] Search indextic Variables()

GOAL_DIS            = 10.0  #                           # [m] goal distance
STOP_SPEED          = 10 / 3.6                          # [m/s] stop speed
TARGET_SPEED        = 72 / 3.6                         # [m/s] target speed
DL                  = 1.0                               # course tick
N_IND_SEARCH        = 10                                # Search index number
MAX_SPEED           = 261.0 / 3.6                       # maximum speed [m/s]  # 261.0 km/h
MIN_SPEED           = 0                                 # minimum speed [m/s]
MAX_STEER           = np.deg2rad(35.0)                  # maximum steering angle [rad](35.0)
MAX_DSTEER          = np.deg2rad(30.0)                  # maximum steering speed [rad/s](30.0)
# MAX_ACCEL           = 3.2                               # maximum accel [m/ss]

""" Vehicle Parameters for plot_car() """
LENGTH              = 4.724  # [m]
WIDTH               = 1.933  # [m]
BACKTOWHEEL         = 1.0  # [m]
WHEEL_LEN           = 0.3  # [m]  # 20 inch
WHEEL_WIDTH         = 0.2  # [m]
TREAD               = 1.584  # [m]
OFFSET              = 25
PLOT_CAR_TICK       = 0.1

""" MSG """
MSG_HEADER          = "[Moral Machine]"
