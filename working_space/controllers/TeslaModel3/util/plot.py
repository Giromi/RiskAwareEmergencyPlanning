import matplotlib.pyplot as plt
import numpy as np
from lib.convention import *
import math

def is_1st(label, first_iteration):
    return label if first_iteration else None

def plot_init():
    ###################################################
    plt.close("all")
    plt.figure(figsize=(5, 3))
    plt.title(f'Path Planning and Tracking {TARGET_SPEED * 3.6}[km/h]', fontsize=16)
    plt.xlabel("X [m]", fontsize=12)
    plt.ylabel("Y [m]", fontsize=12)
    plt.grid(True)
    plt.axis("equal")  # 축 비율을 동일하게
    ###################################################

def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)

def plot_interval(state, steer, cx, cy, target_index):
    cur_t = state.get_time()
    plt.plot(cx[target_index], cy[target_index], "xg", markersize=10)
    if state.plot_time < cur_t :
        state.plot_time = cur_t + PLOT_CAR_TICK
        plot_car(state.x, state.y, state.yaw, steer)
    plt.plot(state.x, state.y, "*")
    plt.legend(loc="upper right", fontsize=10)      # 여기있어야 라벨 뜸
    # plt.xlim(state.x - OFFSET, state.x + OFFSET)
    # plt.ylim(state.y - OFFSET, state.y + OFFSET)
    plt.pause(0.0001)                               # 이게 없으면 그래프가 멈춤

def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def plot_start_goal(start, goal, first_iteration):
    plt.plot(start[X], start[Y], "bo", 
             markersize=10, label=is_1st("Start", first_iteration))
    plt.plot(goal[X], goal[Y], "yo", 
             markersize=10, label=is_1st("LLM(Collision)", first_iteration))

def plot_rrt_star_path(points_waypoint, first_iteration):
    plt.plot(points_waypoint[1:-1, X], points_waypoint[1:-1, Y],
             'ro', markersize=10, label=is_1st('RRT*(Waypoint)', first_iteration))


def plot_spline2d_path(points_path, first_iteration):
    plt.plot(points_path[1:, X], points_path[1:, Y],
             'cx', markersize=5, label=is_1st('Spline2D(Path)', first_iteration))

def plot_target_point(start, first_iteration):
    plt.plot(start[X], start[Y]
             , "xg", markersize=10, label=is_1st("Target Point", first_iteration))
