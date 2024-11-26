from debug import *
from TEST import *

""" Webots """
from controller import Supervisor   # 차후에 webots on/off할 때 필요
from vehicle import Driver

""" Standard """
import numpy as np
# import matplotlib.pyplot as plt

""" Library """
from lib.dubins_planner import DubinsPlanner
from lib.tesla_state import IdealState, TeslaState, History
from lib.rrt_star_planner import RRTStarPlanner
from lib.mpc_tracker import MPCTracker
from lib.path_handler import PathHanlder
from lib.spline2d_planner import Spline2dPlanner
# from lib.rrt_planner import Node  #TODO

""" Util """
from util.operator import angle_mod, smooth_yaw
from util.map_maker import generate_grid_map
from util.llm import request_to_LLM
from lib.convention import *
from util.plot import plot_init

# from get_information import parse_proto, get_value
def webots_sim(driver, tesla_state):    # <Main 문>
    driver.step()
    tesla_state.update()

    dt = driver.getBasicTimeStep() / 1000 # [s] 늘려야할 수도 있음

    grid_map = np.zeros((700, 700))
    # grid_map = generate_grid_map("data/map.json")
    # grid_map = convert_to_grid_map("data/[Map]_03_height_C_H.txt")

    points_collision = request_to_LLM()
    for cur_collision in points_collision:
        start = np.array([tesla_state.x, tesla_state.y])
        goal = np.array([cur_collision[X], cur_collision[Y]])
        plt.plot(start[X], start[Y], "bo", markersize=10, label="Start")
        plt.plot(goal[X], goal[Y], "yo", markersize=10, label="LLM(Collision)")

        """ RRT* Path Planning """
        rrt_star = RRTStarPlanner(grid_map, start, goal)
        points_waypoint = rrt_star.plan()
        plt.plot(points_waypoint[1:-2, X], points_waypoint[1:-2, Y], "ro", markersize=10, label="RRT*(Waypoint)")

        """ Spline2D Path Planning """
        spline2d_planner = Spline2dPlanner(points_waypoint, 'linear') # 'linear', 'quadratic' 'cubic'
        points_path = spline2d_planner.calculate()
        plt.plot(points_path[1:, X], points_path[1:, Y], "cx", markersize=10, label="Dubins(Path)")

        """ MPC Tracking """
        mpc = MPCTracker(points_path, dt)
        mpc.track(tesla_state)

def webots_sim_dubins(driver, tesla_state):    # <Main 문>
    tesla_state.update()
    dt = driver.getBasicTimeStep() / 1000 # [s] 늘려야할 수도 있음

    points_collision = request_to_LLM()
    for cur_collision in points_collision:
        start = np.array([tesla_state.x, tesla_state.y])
        goal = np.array([cur_collision[X], cur_collision[Y]])
        plt.plot(start[X], start[Y], "bo", markersize=10, label="Start")
        plt.plot(goal[X], goal[Y], "yo", markersize=10, label="LLM(Collision)")

        """ Connect """
        start_point = np.concatenate((start, np.deg2rad([tesla_state.yaw], None)))
        points_waypoint = np.vstack((start_point, np.array(points_collision)))
        plt.plot(points_waypoint[1:-2, X], points_waypoint[1:-2, Y], "ro", markersize=10, label="RRT*(Waypoint)")

        """ Dubins Path Planning """
        path_handler = PathHanlder(points_waypoint, DubinsPlanner)
        # path_handler = PathHanlder(points_waypoint, DubinsPlanner)
        # points_path = path_handler.calculate()
        # plt.plot(points_path[1:, X], points_path[1:, Y], "cx", markersize=10, label="Dubins(Path)")

        """ MPC Tracking """
        mpc = MPCTracker(points_path, dt)
        mpc.track(tesla_state)

def make_situation():
    driver = Driver()   # 차량, 건물 및 object의 객체
    tesla_state = TeslaState(driver, 0)

    tesla_state.update()
    init_x, init_y = tesla_state.x, tesla_state.y

    standard_speed = TARGET_SPEED * 0.95
    print(f'[Moral Machine] : {standard_speed * 3.6:.2f}[km/h] 속도를 감지하겠습니다.')
    tesla_state.set_speed(TARGET_SPEED)
    while driver.step() != -1 and tesla_state.v <= standard_speed: # [km/h]
        tesla_state.update()
        print(f'current speed 1(km/h): {tesla_state.get_speed() * 3.6}')
        print(f'current speed 2(km/h): {tesla_state.get_speed_km_h()}')
    print('[Moral Machine] : 이제부터, 이 핸들은 제껍니다.')
    distance = np.linalg.norm([tesla_state.x - init_x, tesla_state.y - init_y])
    driver.simulationSetMode(driver.SIMULATION_MODE_PAUSE)
    print(f'Accuration distance : {distance}')
    return driver, tesla_state

if __name__ == '__main__':
    plot_init()
    driver, tesla_state = make_situation()
    # webots_sim_dubins(driver, tesla_state)
    webots_sim(driver, tesla_state)
    # TEST_06()


