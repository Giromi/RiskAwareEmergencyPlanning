
""" Standard """
import numpy as np
import matplotlib.pyplot as plt

""" Library """
from lib.dubins_planner import DubinsPlanner
from lib.tesla_state import IdealState
from lib.rrt_star_planner import RRTStarPlanner
from lib.mpc_tracker import MPCTracker
from lib.path_handler import PathHanlder
# from lib.rrt_planner import Node  #TODO

""" Util """
from util.operator import angle_mod, smooth_yaw
from util.map_maker import generate_grid_map
from util.llm import request_to_LLM
from lib.convention import *
from util.plot import plot_init


def main():
    plot_init()
    print(__file__ + " start!!")
    points_collision = np.array([ [0.0, 0.0, np.deg2rad(0)],
                             [30.0, 5.0, np.deg2rad(15)],
                             [60.0, -3.0, np.deg2rad(-15)],
                             [90.0, 8.0, np.deg2rad(20)],
                             [120.0, -10.0, np.deg2rad(-20)],
                             [150.0, 12.0, np.deg2rad(25)],
                             [180.0, -15.0, np.deg2rad(-30)],
                             [210.0, 10.0, np.deg2rad(10)],
                             [240.0, -5.0, np.deg2rad(-10)],
                             [270.0, 3.0, np.deg2rad(5)],
                             [300.0, -8.0, np.deg2rad(-5)],
                             [330.0, 15.0, np.deg2rad(30)],
                             [360.0, -20.0, np.deg2rad(-25)],
                             [390.0, 18.0, np.deg2rad(20)],
                             [420.0, -12.0, np.deg2rad(-20)],
                             [450.0, 10.0, np.deg2rad(15)],
                             [480.0, -7.0, np.deg2rad(-10)],
                             [510.0, 5.0, np.deg2rad(5)],
                             [540.0, -3.0, np.deg2rad(-5)],
                             [570.0, 2.0, np.deg2rad(3)],
                             [600.0, 0.0, np.deg2rad(0)]   ])
    dt = 0.1
    ideal_state = IdealState(dt, x=points_collision[0, X], 
                                 y=points_collision[0, Y], 
                                 yaw=points_collision[0, YAW], v=0)

    start = np.array([points_collision[0, X], points_collision[0, Y]])
    goal = np.array([points_collision[-1, X], points_collision[-1, Y]])

    """ Connect """
    start_point = np.concatenate((start, np.deg2rad([ideal_state.yaw], None)))
    points_waypoint = np.vstack((start_point, np.array(points_collision)))
    plt.plot(start[X], start[Y], "bo", markersize=10, label="Start")
    plt.plot(goal[X], goal[Y], "yo", markersize=10, label="LLM(Collision)")

    """ Dubins Path Planning """
    path_handler = PathHanlder(points_waypoint, DubinsPlanner)
    points_path = path_handler.calculate()
    plt.plot(points_path[1:, X], points_path[1:, Y], "cx", markersize=10, label="Dubins(Path)")

    """ MPC Tracking """
    mpc = MPCTracker(points_path, dt)
    mpc.do_simulation(ideal_state)

if __name__ == '__main__':
    main()
