
""" Standard """

import numpy as np

""" Library """
from lib.rrt_star_planner import RRTStarPlanner

""" Util """
from util.map_maker import generate_grid_map

if __name__ == "__main__":
    # here, update map_data path.
     grid_map = generate_grid_map(
             "data/data.json"
             )

     start_point = (0, len(grid_map[0]) // 2)
     goal_point = (50, len(grid_map[0]) // 2)

     rrt_star = RRTStarPlanner(grid_map, start_point, goal_point, velocity=30)
     path = rrt_star.plan()
     print(path)
     rrt_star.visualize(path)
