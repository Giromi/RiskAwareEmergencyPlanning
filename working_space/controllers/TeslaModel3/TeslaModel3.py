# 2024.11.17 Capdi
import numpy as np
from controller import Supervisor   # 차후에 webots on/off할 때 필요
from vehicle import Driver
import matplotlib.pyplot as plt
# from get_information import parse_proto, get_value

from lib.tesla_state import IdealState, TeslaState, History
# from lib.rrt_star_planner import RRTStarPlanner
from lib.mpc_tracker import MPCTracker
from utils.operator import angle_mod, smooth_yaw
# from lib.rrt_planner import Node  #TODO
from utils.debug import get_my_dubins_course
from lib.convention import Waypoint, X, Y, YAW

def show_history(cx, cy, tesla_history):
    plt.close("all")
    plt.subplots()
    plt.plot(cx, cy, "-r", label="Reference Path")
    plt.plot(tesla_history.x, tesla_history.y, "-b", label="Tracking Path")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()


    plt.subplots()
    plt.plot(tesla_history.t, tesla_history.v, "-r", label="speed")
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.show()



def main(driver, collision_point_list):    # <Main 문>
    ###################################################
    DT = driver.getBasicTimeStep() / 1000 # [s]
    DL = 1.0  # course tick
    ###################################################

    """ Consturctor """

    # TARGET_SPEED = 108 / 3.6  # [m/s] target speed
    TARGET_SPEED = 36 / 3.6  # [m/s] target speed

    tesla_history = History()
    tesla_state = TeslaState(driver, DT)
    # ideal_history = History()
    # ideal_state = IdealState()
    mpc_tracker = MPCTracker(DT)

    """ LLM  """
    #LLM에게 요청(info.txt, current_pos)


    # start = Waypoint(tesla_state.x, tesla_state.y, tesla_state.yaw)
    # goal = [collision_point_list[-1].x, collision_point_list[-1].y]


    # initial_state = IdealState(start.x, start.y, start.yaw, 0.0)

    initial_position = tesla_state.get_position()
    path = np.vstack([start, collision_point_list])

    # tesla_history.append(tesla_state)

    # print("\nSpeed : ", driver.getCurrentSpeed())              # 속력 (m/s)

    MAX_TIME = 500.0  # max simulation time
    # stopwatch = 0.0
    for collision_point in collision_point_list:
        tesla_state.update()
        # start = Waypoint(tesla_state.x, tesla_state.y, tesla_state.yaw)
        start = np.array([initial_position[X], initial_position[Y], initial_position[YAW]])

        # path = np.vstack([start, collision_point])

        cx, cy, cyaw = get_my_dubins_course(path)
        initial_state = IdealState(DT, x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)
        target_index, _ = mpc_tracker.calculate_nearest_index(initial_state, cx, cy, cyaw, 0)
        sp = mpc_tracker.calculate_speed_profile(cx, cy, cyaw, TARGET_SPEED)

        # print('-'*50)
        # print('cx : ', cx)
        # print('cy : ', cy)
        # print('cyaw : ', cyaw)
        # print('sp : ', sp)
        # print("Initial State : ", initial_state)
        # print("Target Index : ", target_index)
        # print('-'*50)
        
        goal = [cx[-1], cy[-1]]
        odelta, oa = None, None

        cyaw = smooth_yaw(cyaw)
        while MAX_TIME >= driver.getTime():
            x_ref, target_index, d_ref = mpc_tracker.calculate_ref_trajectory(
                            tesla_state, cx, cy, cyaw, sp, DL, target_index)
            x_cur = [tesla_state.x, tesla_state.y , tesla_state.v, tesla_state.yaw]
            oa, odelta, ox, oy, oyaw, ov = mpc_tracker.iterative_linear_control(x_ref, x_cur, d_ref, oa, odelta)
            # x_cur_ideal = [ideal_state.x, ideal_state.x, ideal_state.y, ideal_state.v, ideal_state.yaw], 
            # oa, odelta, ox, oy, oyaw, ov = mpc_tracker.control(x_ref, x_cur_ideal, d_ref, oa, odelta)
            d_index, a_index = 0.0, 0.0
            if odelta is not None:
                d_index, a_index = odelta[0], oa[0]
                tesla_state.update(d_index)
                # ideal_state.update(a_index, d_index)
            # stopwatch += DT
            tesla_history.append(driver.getTime(), tesla_state)
            # ideal_history.append(ideal_state)

            if mpc_tracker.check_goal(goal, target_index, len(cx)):
                return tesla_history, True
    return tesla_history, False

def check_time():
    old_at = driver.getTime()
    while driver.step() != -1:  
        now = driver.getTime()
        print(now - old_at) # 0.008
        old_at = driver.getTime()

def show_history(collision_point_list, tesla_history):
    start = Waypoint(tesla_history.x_list[0], tesla_history.y_list[0], tesla_history.yaw_list[0])
    path = np.vstack([start, collision_point_list])
    cx, cy, cyaw = get_my_dubins_course(path)
    plt.close("all")
    plt.subplots()
    plt.plot(cx, cy, "-r", label="Reference Path")
    plt.plot(tesla_history.x, tesla_history.y, "-b", label="Tracking Path")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()

    plt.subplots()
    plt.plot(tesla_history.t, tesla_history.v, "-r", label="speed")
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.show()

def request_collision_point_list_to_LLM():
    return [ np.array([30.0, 5.0, np.deg2rad(15)]),
             np.array([60.0, -3.0, np.deg2rad(-15)]),
             np.array([90.0, 8.0, np.deg2rad(20)]),
             np.array([120.0, -10.0, np.deg2rad(-20)]),
             np.array([150.0, 12.0, np.deg2rad(25)]),
             np.array([180.0, -15.0, np.deg2rad(-30)]),
             np.array([210.0, 10.0, np.deg2rad(10)]),
             np.array([240.0, -5.0, np.deg2rad(-10)]),
             np.array([270.0, 3.0, np.deg2rad(5)]),
             np.array([300.0, -8.0, np.deg2rad(-5)]),
             np.array([330.0, 15.0, np.deg2rad(30)]),
             np.array([360.0, -20.0, np.deg2rad(-25)]),
             np.array([390.0, 18.0, np.deg2rad(20)]),
             np.array([420.0, -12.0, np.deg2rad(-20)]),
             np.array([450.0, 10.0, np.deg2rad(15)]),
             np.array([480.0, -7.0, np.deg2rad(-10)]),
             np.array([510.0, 5.0, np.deg2rad(5)]),
             np.array([540.0, -3.0, np.deg2rad(-5)]),
             np.array([570.0, 2.0, np.deg2rad(3)]),
             np.array([600.0, 0.0, np.deg2rad(0)]) ]

if __name__ == '__main__':
    driver = Driver()   # 차량, 건물 및 object의 객체
    simulation_flag = True 
    collision_point_list = request_collision_point_list_to_LLM()
    while driver.step() != -1 and simulation_flag == True:  # iter 사실상 1번
        tesla_history, simulation_flag = main(driver, collision_point_list)
        show_history(collision_point_list, tesla_history)

