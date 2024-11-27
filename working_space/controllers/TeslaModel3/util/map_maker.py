import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import ListedColormap, BoundaryNorm


def generate_grid_map(map_file):
    # Load data from JSON file
    with open(map_file, "r") as f:
        data = json.load(f)

    # Set grid dimensions and resolution
    GRID_RESOLUTION = 0.1  # 0.1 meter per cell
    GRID_X_MIN, GRID_X_MAX = -20, 140
    GRID_Y_MIN, GRID_Y_MAX = -80, 80

    # Calculate grid size
    grid_width = int((GRID_X_MAX - GRID_X_MIN) / GRID_RESOLUTION)
    grid_height = int((GRID_Y_MAX - GRID_Y_MIN) / GRID_RESOLUTION)

    # Initialize combined grid map for all obstacles
    grid_map_combined = np.zeros((grid_height, grid_width))

    # Function to convert world coordinates to grid indices
    def world_to_grid(x, y):
        grid_x = int((x - GRID_X_MIN) / GRID_RESOLUTION)
        grid_y = int((y - GRID_Y_MIN) / GRID_RESOLUTION)
        return grid_x, grid_y

    # Function to mark an area occupied in the grid map
    def mark_occupied(grid, pos, size, ori, category_value):
        if isinstance(size, (int, float)):
            dx = dy = dz = size
        elif len(size) == 3:
            dx, dy, dz = size
        x, y, z = pos

        # Extract orientation matrix and calculate angle (assuming 2D rotation on xy plane)
        angle = np.arctan2(ori[1], ori[0])
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)

        # Calculate the corners of the rectangle
        half_dx = dx / 2
        half_dz = dz / 2
        corners = [
            [-half_dx, -half_dz],
            [half_dx, -half_dz],
            [half_dx, half_dz],
            [-half_dx, half_dz],
        ]

        # Rotate corners based on orientation and translate to position
        rotated_corners = []
        for corner in corners:
            rotated_x = cos_angle * corner[0] - sin_angle * corner[1] + x
            rotated_y = sin_angle * corner[0] + cos_angle * corner[1] + y
            rotated_corners.append((rotated_x, rotated_y))

        # Find the bounding box of the rotated rectangle
        min_x = min(c[0] for c in rotated_corners)
        max_x = max(c[0] for c in rotated_corners)
        min_y = min(c[1] for c in rotated_corners)
        max_y = max(c[1] for c in rotated_corners)

        # Mark the occupied cells in the grid
        for i in np.arange(min_x, max_x, GRID_RESOLUTION):
            for j in np.arange(min_y, max_y, GRID_RESOLUTION):
                grid_x, grid_y = world_to_grid(i, j)
                if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
                    grid[grid_y, grid_x] = category_value

    # Mark sidewalk in the grid first
    for value in data.get("sidewalk_dict", {}).values():
        mark_occupied(grid_map_combined, value["pos"], value["size"], value["ori"], 1)

    # Mark all other objects (buildings, cars, trees, humans) in the grid
    for obj_dict, category_value in zip(
        [
            data.get("building_dict", {}),
            data.get("car_dict", {}),
            data.get("tree_dict", {}),
            data.get("human_dict", {}),
        ],
        [2, 3, 4, 5],
    ):
        for value in obj_dict.values():
            if "radius" in value:  # For trees
                x, y, _ = value["pos"]
                radius = value["radius"]
                size = [radius * 2, 0, radius * 2]
                ori = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # No rotation
                mark_occupied(grid_map_combined, [x, y, 0], size, ori, category_value)
            else:  # For buildings, cars, and humans
                mark_occupied(
                    grid_map_combined,
                    value["pos"],
                    value["size"],
                    value["ori"],
                    category_value,
                )

    return grid_map_combined
