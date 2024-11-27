from util.debug import *
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
from util.simulation import request_to_LLM, make_situation
from lib.convention import *
from util.plot import plot_init

def check_contact_to_ground(driver, tesla_state):
    old_t = tesla_state.get_time()
    while (driver.step() != -1):
        tesla_state.update()
        if (tesla_state.get_front_z_position() - tesla_state.z <= 0.005):
            continue
        if (tesla_state.get_time() - old_t < 1.0):
            old_t = tesla_state.get_time()
            continue
        break

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
        # points_waypoint = np.vstack((start, np.array(points_collision)))
        # points_waypoint = np.hstack((points_waypoint, np.zeros((points_waypoint.shape[0], 1))))
        # plt.plot(points_waypoint[:, X], points_waypoint[:, Y], "ro", markersize=10, label="RRT*(Waypoint)")


        """ Spline2D Path Planning """
        spline2d_planner = Spline2dPlanner(points_waypoint, 'linear') # 'linear', 'quadratic' 'cubic'
        points_path = spline2d_planner.calculate()
        plt.plot(points_path[1:, X], points_path[1:, Y], "cx", markersize=5, label="Spline2D(Path)")

        """ MPC Tracking """
        plt.plot(points_path[0, X], points_path[0, Y], "xg", markersize=10, label="Target Point")
        mpc = MPCTracker(points_path, dt)
        mpc.track(tesla_state)

        check_contact_to_ground(tsla_state)

# from get_information import parse_proto, get_value
def webots_ideal(driver, ideal_state):    # <Main 문>
    driver.step()
    dt = driver.getBasicTimeStep() / 1000 # [s] 늘려야할 수도 있음
    grid_map = np.zeros((700, 700))
    points_collision = request_to_LLM()
    for cur_collision in points_collision:
        """ Connect """
        start = np.array([ideal_state.x, ideal_state.y])
        goal = np.array([cur_collision[X], cur_collision[Y]])
        plt.plot(start[X], start[Y], "bo", markersize=10, label="Start")
        plt.plot(goal[X], goal[Y], "yo", markersize=10, label="LLM(Collision)")

        """ RRT* Path Planning """ # NOT YET
        points_waypoint = np.vstack((start, np.array(points_collision)))
        points_waypoint = np.hstack((points_waypoint, np.zeros((points_waypoint.shape[0], 1))))
        plt.plot(points_waypoint[:, X], points_waypoint[:, Y], "ro", markersize=10, label="RRT*(Waypoint)")

        """ Spline2D Path Planning """
        spline2d_planner = Spline2dPlanner(points_waypoint, 'linear') # 'linear', 'quadratic' 'cubic'
        points_path = spline2d_planner.calculate()
        plt.plot(points_path[1:, X], points_path[1:, Y], "cx", markersize=5, label="Dubins(Path)")

        """ MPC Tracking """
        plt.plot(points_path[0, X], points_path[0, Y], "xg", markersize=10, label="Target Point")
        mpc = MPCTracker(points_path, dt)
        mpc.do_simulation(driver, ideal_state)

if __name__ == '__main__':
    plot_init()
    driver, tesla_state, ideal_state = make_situation()
    webots_sim(driver, tesla_state)
    # webots_ideal(driver, ideal_state)
    # TEST_07()
