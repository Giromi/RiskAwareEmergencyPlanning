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
from util.plot import is_1st, plot_init, plot_start_goal, plot_rrt_star_path, \
                         plot_spline2d_path, plot_target_point

def check_contact_to_ground(driver, tesla_state):
    old_t = tesla_state.get_time()
    while (driver.step() != -1):
        tesla_state.update()
        if (tesla_state.get_front_z_position() - tesla_state.z <= 0.005):
            print('Contact to Ground')
            continue
        if (tesla_state.get_time() - old_t < 1.0):
            old_t = tesla_state.get_time()
            print('Waiting for 1 sec...')
            continue
        break

def webots_sim():
    driver = Driver()   # 차량, 건물 및 object의 객체
    dt = driver.getBasicTimeStep() / 1000
    tesla_state = make_situation(driver, dt)
    dt = driver.getBasicTimeStep() / 1000 # [s] 늘려야할 수도 있음
    tesla_state.update()
    # grid_map = generate_grid_map("data/data.json")

    grid_map = np.zeros((500, 500))
    first_iteration = True
    i = 0
    points_collision = request_to_LLM()
    while i < len(points_collision):
        start = np.array([tesla_state.x, tesla_state.y])
        goal = np.array([points_collision[i, X], points_collision[i, Y]])

        """ RRT* Path Planning """
        # rrt_star = RRTStarPlanner(grid_map, start, goal, tesla_state.v)
        # points_waypoint = rrt_star.plan()
        # if points_waypoint is None:
        #     continue

        # """ 디버깅 용도 """
        middle_point = np.array([(start[X] + points_collision[i, X]) / 2, 
                                 (start[Y] + points_collision[i, Y]) / 2])
        points_waypoint = np.vstack([start, middle_point, points_collision[i]])
        points_waypoint = np.hstack((points_waypoint, np.zeros((points_waypoint.shape[0], 1))))

        """ Spline2D Path Planning """
        spline2d_planner = Spline2dPlanner(points_waypoint, tesla_state.v * dt, 'linear')
        points_path = spline2d_planner.calculate()

        """ Plotting """
        plot_start_goal(start, goal, first_iteration)
        plot_rrt_star_path(points_waypoint, first_iteration)
        plot_spline2d_path(points_path, first_iteration)
        plot_target_point(start, first_iteration)

        """ MPC Tracking """
        mpc = MPCTracker(points_path, dt)
        mpc.track(tesla_state)
        # check_contact_to_ground(driver, tesla_state)

        i += 1
        first_iteration = False

# from get_information import parse_proto, get_value
def webots_ideal():    # <Main 문>
    driver = Driver()   # 차량, 건물 및 object의 객체
    dt = driver.getBasicTimeStep() / 1000
    tesla_state = make_situation(driver, dt)
    ideal_state = IdealState(dt, x=tesla_state.x, y=tesla_state.y, 
                             yaw=tesla_state.yaw, v=tesla_state.v)

    grid_map = np.zeros((500, 500))
    # grid_map = generate_grid_map('data/data.json')
    points_collision = request_to_LLM()

    i = 0
    first_iteration = True
    while i < len(points_collision):
        cur_collision = points_collision[i]
        """ Connect """
        start = np.array([ideal_state.x, ideal_state.y])
        goal = np.array([cur_collision[X], cur_collision[Y]])

        """ RRT* Path Planning """
        rrt_star = RRTStarPlanner(grid_map, start, goal, ideal_state.v)
        points_waypoint = rrt_star.plan()
        if points_waypoint is None:
            continue

        """ 디버깅 용도 """
        points_waypoint = np.vstack((start, points_collision[i]))
        points_waypoint = np.hstack((points_waypoint, np.zeros((points_waypoint.shape[0], 1))))

        """ Spline2D Path Planning """
        spline2d_planner = Spline2dPlanner(points_waypoint, ideal_state.v * dt, 'linear') # 'linear', 'quadratic' 'cubic'
        points_path = spline2d_planner.calculate()

        """ Plotting """
        plot_start_goal(start, goal, first_iteration)
        plot_rrt_star_path(points_waypoint, first_iteration)
        plot_spline2d_path(points_path, first_iteration)
        plot_target_point(start, first_iteration)

        """ MPC Tracking """
        mpc = MPCTracker(points_path, dt)
        mpc.do_simulation(driver, ideal_state)

        i += 1
        first_iteration = False

if __name__ == '__main__':
    plot_init()
    webots_sim()
    # webots_ideal(driver, dt, ideal_state)
    # TEST_09()
