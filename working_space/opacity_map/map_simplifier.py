import json
import numpy as np

GRID_RESOLUTION = 1  # 1 meter per cell
GRID_X_MIN, GRID_X_MAX = -50, 50
GRID_Y_MIN, GRID_Y_MAX = -80, 80
X, Y, = 0, 1
W, H = 0, 2

def mark_occupied(name, file, pos, size, ori):
    dx, dy, dz = size #TODO: Check if dz is the height
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


    file.write(f'{name} : ({min_x}, {min_y}, {max_x}, {max_y})\n')
    
    

def simplify_grid_map(json_file):

    with open(json_file, "r") as f:
        data = json.load(f)

    # Calculate grid size
    grid_width = int((GRID_X_MAX - GRID_X_MIN) / GRID_RESOLUTION)
    grid_height = int((GRID_Y_MAX - GRID_Y_MIN) / GRID_RESOLUTION)

    output = open('output.txt', 'w')
    output.write(f'Map size: {grid_width}, {grid_height}\n\n')
    # output.write('# min_x, min_y, max_x, max_y\n')
    output.write('# x, y, w, h\n')

    for obj_dict in [data["building_dict"], data["car_dict"], data["tree_dict"]]:

        for key, value in obj_dict.items():
            pos, size, ori = [], [], []
            try:
                pos = value["pos"]
                size = value["size"]
                ori = value["ori"]
            except:
                if "radius" in value:  # For trees
                    pos = value["pos"]
                    size = [value["radius"] * 2, 0, value['radius'] * 2]
                    ori = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            output.write(f'{key} : ({pos[X]}, {pos[Y]}, {size[W]}, {size[H]})\n')
            # mark_occupied(key, output, pos, size, ori)
    output.close()


if __name__ == "__main__":
    simplify_grid_map('data.json')

