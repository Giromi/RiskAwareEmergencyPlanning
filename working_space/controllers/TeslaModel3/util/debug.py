# 2024.11.17 Capdi


""" Webots """
from lib.tesla_state import IdealState, TeslaState, History

from numpy.typing import NDArray
from typing import Type
import numpy as np
from lib.dubins_planner import DubinsPlanner
import matplotlib.pyplot as plt

def dprint(val):
    print(f'{val = }')


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

    mpc = MPCTracker(points_path, dt)
    mpc.do_simulation(tesla_state)



color_map = {
    '0': (255/255, 255/255, 255/255),  # White for empty space
    '1': (211/255, 211/255, 211/255),  # Light silver for sidewalk
    '5': (220/255, 220/255, 220/255),  # Light gray for low buildings
    '6': (211/255, 211/255, 211/255),  # Middle range gray for middle buildings
    '7': (200/255, 200/255, 200/255),  # Darker gray for middle buildings
    '8': (192/255, 192/255, 192/255),  # Dark gray for tall buildings
    '9': (169/255, 169/255, 169/255),  # Dark gray for tallest buildings
    'M': (0/255, 0/255, 255/255),      # Blue for my vehicle
    'C': (255/255, 165/255, 0/255),    # Orange for other vehicles
    'T': (34/255, 139/255, 34/255),    # Green for trees
    'G': (192/255, 192/255, 192/255),  # Dim silver for guardrails
    'H': (65/255, 41/255, 35/255)      # Brown for humans
}

def convert_to_grid_map(file_path: str):
    with open(file_path, 'r') as file:
        raw_data = file.readlines()
    grid = [list(line.strip()) for line in raw_data]
    image = np.zeros((len(grid), len(grid[0]), 3))
    for i, row in enumerate(grid):
        for j, cell in enumerate(row):
            image[i, j] = color_map.get(cell, (0, 0, 0))  # Default to black

    return image.reshape(len(grid), len(grid[0]))



def grid_plot(image):
    # image = convert_to_grid_map(file_path)
    plt.figure(figsize=(12, 12))
    plt.imshow(image, origin='lower', cmap='gray')

    plt.xlabel("X-axis (grid)")
    plt.ylabel("Y-axis (grid)")
    plt.title("Grid Map Visualization")
    plt.show()
