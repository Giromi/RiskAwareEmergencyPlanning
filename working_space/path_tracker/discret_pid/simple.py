
import time
from utils.operation import angle_mod
import numpy as np
import math
import matplotlib.pyplot as plt
from capdilib.controller import DiscretPIDController
from capdilib.dubins_path_planner import DubinsPathPlanner
from capdilib.convention import Waypoint
from capdilib.state_states import State, State_list
from capdilib.path_handler import PathHanlder
from capdilib.target_course import TargetCourse
from utils.plot import plot_arow
from capdilib.convention import X, Y, YAW

# Parameters
show_animation = True

def calculate_error(state, trajectory, pind):
    target_index = trajectory.search_target_index(state)

    if pind >= target_index:
        target_index = pind

    if target_index < len(trajectory.tx):
        target_x = trajectory.tx[target_index]
        target_y = trajectory.ty[target_index]
    else:  # toward goal
        target_x = trajectory.tx[-1]
        target_y = trajectory.ty[-1]
        target_index = len(trajectory.tx) - 1

    distance_error = state.cal_distance(target_x, target_y)
    print(f'target_x: {target_x},\tstate.x: {state.x}')
    print(f'target_y: {target_y},\tstate.y: {state.y}')
    direction_error = state.cal_direction(target_x, target_y)
    sign = -1 if direction_error > 0 else 1
    return sign * distance_error, target_index

# r_min 고려해야 함 
# TODO control input 마치 24V saturation
def main():
    ##################################################
    # LLM
    path: list[Waypoint] = [ np.array([0.0, 0.0, np.deg2rad(0.)]), # LSL
                             np.array([61.4, 5.2, np.deg2rad(45.)]), # RSR
                             np.array([112.1, 7.2, np.deg2rad(0.)]) ]
    SPEED = 100.0 # [km/h]
    END_TIME = 100.0  # max simulation time
    # K_P, K_I, K_D = 0.01, 0.0, 0.0            # Case 1
    # K_P, K_I, K_D = 0.025, 0.0, 0.0           # Case 2
    # K_P, K_I, K_D = 0.05, 0.0, 0.0            # Case 3
    # K_P, K_I, K_D = 0.075, 0.0, 0.0           # Case 4
    # K_P, K_I, K_D = 0.10, 0.0, 0.0            # Case 5
    # K_P, K_I, K_D = 0.15, 0.0, 0.0              # Case 6

    K_P, K_I, K_D = 0.50, 0.0, 0.0              # Case 6


    # K_P, K_I, K_D = 0.05, 0.00, 0.1          # Case 7
    # K_P, K_I, K_D = 0.05, 0.00, 0.12          # Case 7
    # K_P, K_I, K_D = 0.05, 0.00, 0.15          # Case 7
    # K_P, K_I, K_D = 0.05, 0.00, 0.156          # Case 7
    # K_P, K_I, K_D = 0.05, 0.00, 0.157          # Case 7
    # K_P, K_I, K_D = 0.05, 0.00, 0.2          # Case 7
    ##################################################

    dt = 0.1
    target_speed = SPEED / 3.6  # [m/s]
    """ Example 1: Sinusoidal path """
    # target_path_x = np.arange(0, 50, 0.5)
    # target_path_y = [math.sin(xp / 5.0) * xp / 2.0 for xp in target_path_x]
    # print('target_path_x size :', target_path_x.shape)
    # state = State(dt, v=target_speed, 
    #                   x=target_path_x[0], y=target_path_y[0], yaw=0.0)

    """ Example 2: Dubins path """
    path_handler = PathHanlder(path, DubinsPathPlanner)

    target_path_x, target_path_y, target_path_yaw = path_handler.calculate_path()
    state = State(dt, v=target_speed, x=target_path_x[0], 
                                      y=target_path_y[0], 
                                      yaw=target_path_yaw[0])

    # initial state
    lastIndex = len(target_path_x) - 1
    check_time = 0.0
    state_list = State_list()
    state_list.append(check_time, state)

    # pure pursuit > Lf = k * v + Lfc # PID 
    target_course = TargetCourse(target_path_x, target_path_y)
    target_index = target_course.search_target_index(state)
    controller = DiscretPIDController(Kp=K_P, Ki=K_I, Kd=K_D, T=dt)


    print('target_path_x size :', target_path_x.shape)
    print(f'target_index: {target_index}')
    print(f'taget_path_x : {target_path_x}')
    print(f'taget_path_y : {target_path_y}')
    start = time.time()
    total_error = 0.0
    while END_TIME >= check_time and lastIndex > target_index:
        cur_error, target_index = calculate_error(state, target_course, target_index)
        total_error += cur_error ** 2
        print(f'>>> target_index: {target_index}, cur_error: {cur_error}\n')
        # cur_input = controller.P(cur_error)
        # cur_input = controller.PI(cur_error)
        tick = time.time() - start
        cur_input = controller.PD(cur_error)
        print(f'tick: {tick}[s]')
        start = time.time()
        # cur_input = controller.PID(cur_error)
        state.update(None, cur_input)  # Control vehicle

        check_time += dt
        state_list.append(check_time, state)

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plot_arrow(state.x, state.y, state.yaw)
            plt.plot(target_path_x, target_path_y, "-r", label="course")
            plt.plot(state_list.x, state_list.y, "-b", label="trajectory")
            plt.plot(target_path_x[target_index], target_path_y[target_index], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    plt.close("all")
    # Test
    assert lastIndex >= target_index, "Cannot goal"

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
    RSE = (total_error / (len(state_list.x) - 2)) ** 0.5
    if show_animation:  # pragma: no cover
        fig.canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        ax1.plot(target_path_x, target_path_y, "-r", label="course")        # 경로 표시
        ax1.plot(state_list.x, state_list.y, "-b", label="trajectory")  # 주행 궤적 표시
        ax1.legend()
        ax1.set_xlabel("x [m]")
        ax1.set_ylabel("y [m]")
        ax1.axis("equal")
        ax1.grid(True)
        ax1.set_title("Vehicle Trajectory")
        ax1.text(0.5, 0.04, f'RSE : {RSE:.2f}', 
                             transform=ax1.transAxes, ha='center', va='bottom', fontweight='bold')
        ax1.text(0.05, 0.93, f'Kp : {K_P}, Ki : {K_I}, Kd : {K_D}', 
                             transform=ax1.transAxes, ha='left', va='top', fontweight='bold')

        ax2.plot(state_list.t, [iv * 3.6 for iv in state_list.v], "-r")  # 속도 변화 표시
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Speed [km/h]")
        ax2.grid(True)
        ax2.set_title("Speed over Time")

        plt.tight_layout()
        plt.savefig('result.png')
        plt.show()

if __name__ == '__main__':
    main()
