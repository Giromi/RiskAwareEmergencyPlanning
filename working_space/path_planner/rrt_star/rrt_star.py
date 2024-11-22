import numpy as np
import matplotlib.pyplot as plt
import random
import math
import time
from map_maker import generate_grid_map

TESLA_MIN_RADIUS = 11.70432 # meters
CAR_LENGTH = 5  # meters
CAR_WIDTH = 2  # meters


class Node:
    def __init__(self, x, y, theta=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.parent = None


class RRTStar:
    def __init__(
        self,
        grid,
        start,
        goal,
        velocity=30, # unit [m/s]
        goal_radius=10,
        max_iter=10000,
        min_turn_radius=TESLA_MIN_RADIUS,
    ):
        alpha =  3.6 // 10  # 속도에 반비례하여 step size[m] 결정하기 위한 하이퍼 파라미터
        self.grid = grid
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.step_size = velocity * alpha
        self.goal_radius = goal_radius
        self.max_iter = max_iter
        self.min_turn_radius = min_turn_radius
        self.nodes = [self.start]
        self.grid_height, self.grid_width = grid.shape

    def get_random_node(self):
        x = random.randint(0, self.grid_width - 1)
        y = random.randint(0, self.grid_height - 1)
        theta = random.uniform(-math.pi, math.pi)
        return Node(x, y, theta)

    def get_nearest_node(self, random_node):
        return min(
            self.nodes,
            key=lambda node: math.hypot(node.x - random_node.x, node.y - random_node.y),
        )

    def is_collision_free(self, node1, node2):
        x1, y1 = node1.x, node1.y
        x2, y2 = node2.x, node2.y
        points = zip(
            np.linspace(x1, x2, num=20).astype(int),
            np.linspace(y1, y2, num=20).astype(int),
        )
        for x, y in points:
            if not self.is_within_bounds(x, y) or self.grid[y][x] == 1:
                return False

        # Check vehicle size constraints at the new node position
        if not self.is_car_collision_free(node2):
            return False

        return True

    def is_within_bounds(self, x, y):
        return 0 <= x < self.grid_width and 0 <= y < self.grid_height

    def is_car_collision_free(self, node):
        # Consider the car's rectangular dimensions
        for dx in range(-CAR_LENGTH // 2, CAR_LENGTH // 2 + 1):
            for dy in range(-CAR_WIDTH // 2, CAR_WIDTH // 2 + 1):
                x = int(node.x + dx * math.cos(node.theta) - dy * math.sin(node.theta))
                y = int(node.y + dx * math.sin(node.theta) + dy * math.cos(node.theta))
                if not self.is_within_bounds(x, y) or self.grid[y][x] == 1:
                    return False
        return True

    def get_new_node(self, nearest_node, random_node):
        theta = math.atan2(
            random_node.y - nearest_node.y, random_node.x - nearest_node.x
        )
        new_theta = theta
        angle_diff = abs(new_theta - nearest_node.theta)

        if angle_diff > math.pi:
            angle_diff = 2 * math.pi - angle_diff

        # Check if the turning radius constraint is satisfied
        if angle_diff > (self.step_size / self.min_turn_radius):
            return None

        new_x = int(nearest_node.x + self.step_size * math.cos(theta))
        new_y = int(nearest_node.y + self.step_size * math.sin(theta))
        new_node = Node(new_x, new_y, new_theta)
        new_node.parent = nearest_node
        return new_node

    def rewire(self, new_node):
        for node in self.nodes:
            if node == new_node:
                continue
            distance = math.hypot(node.x - new_node.x, node.y - new_node.y)
            if distance < self.step_size * 2 and self.is_collision_free(new_node, node):
                if self.get_cost(new_node) + distance < self.get_cost(node):
                    node.parent = new_node

    def get_cost(self, node):
        cost = 0
        while node.parent is not None:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent
        return cost

    def plan(self):
        start_time = time.time()
        for _ in range(self.max_iter):
            random_node = self.get_random_node()
            nearest_node = self.get_nearest_node(random_node)
            new_node = self.get_new_node(nearest_node, random_node)

            if new_node is None:
                continue

            if self.is_within_bounds(new_node.x, new_node.y):
                if self.is_collision_free(nearest_node, new_node):
                    self.nodes.append(new_node)
                    self.rewire(new_node)

                    if (
                        math.hypot(new_node.x - self.goal.x, new_node.y - self.goal.y)
                        <= self.goal_radius
                    ):
                        self.goal.parent = new_node
                        self.nodes.append(self.goal)
                        end_time = time.time()
                        print(f"Path found in {end_time - start_time:.2f} seconds")
                        return self.get_path()
        end_time = time.time()
        print(f"No path found in {end_time - start_time:.2f} seconds")
        return None

    def get_path(self):
        path = []
        node = self.goal
        while node is not None:
            path.append((node.x, node.y, node.theta))
            node = node.parent
        return path[::-1]

    def visualize(self, path=None):
        plt.imshow(self.grid, cmap="gray")
        plt.gca().invert_yaxis()
        plt.plot(self.start.x, self.start.y, "go", markersize=10, label="Start")
        plt.plot(self.goal.x, self.goal.y, "ro", markersize=10, label="Goal")

        for node in self.nodes:
            if node.parent is not None:
                plt.plot(
                    [node.x, node.parent.x], [node.y, node.parent.y], "y-", alpha=0.5
                )

        if path:
            path = np.array(path)
            plt.plot(path[:, 0], path[:, 1], "b-", linewidth=2, label="Path")

        plt.legend()
        plt.show()


# if __name__ == "__main__":
#     # here, update map_data path.
#     grid_map = generate_grid_map("data.json")

#     start_point = (0, len(grid_map[0]) // 2)
#     goal_point = (len(grid_map[0]) - 10, len(grid_map[0]) // 2)

#     rrt_star = RRTStar(grid_map, start_point, goal_point, velocity=108)
#     path = rrt_star.plan()
#     print(path)
#     rrt_star.visualize(path)
