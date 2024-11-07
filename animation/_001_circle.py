import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import Iterable
from matplotlib.artist import Artist
#
# # 초기 설정
# SPEED = 10.0  # [km/h]
# END_TIME = 1000
# # K_P, K_I, K_D = 0.05, 0.0, 0.0
# K_P, K_I, K_D = 0.05, 0.0, 0.15
# dt = 0.1
# target_speed = SPEED / 3.6  # [m/s]
#
# path = [np.array([0.0, 0.0, np.deg2rad(0.)]),
#         np.array([61.4, 5.2, np.deg2rad(45.)]),
#         np.array([112.1, 7.2, np.deg2rad(0.)])]
#
# # 경로와 초기 상태 정의
# path_handler = PathHanlder(path, DubinsPathPlanner)
# target_path_x, target_path_y, target_path_yaw = path_handler.calculate_path()
# state = State(dt, v=target_speed, x=target_path_x[0], y=target_path_y[0], yaw=target_path_yaw[0])
#
# state_list = State_list()
# state_list.append(0.0, state)
# target_course = TargetCourse(target_path_x, target_path_y)
# target_index = target_course.search_target_index(state)
# controller = DiscretPIDController(Kp=K_P, Ki=K_I, Kd=K_D, T=dt)
# total_error = 0.0
#
# def calculate_error(state, trajectory, pind):
#     target_index = trajectory.search_target_index(state)
#     if pind >= target_index:
#         target_index = pind
#
#     if target_index < len(trajectory.tx):
#         target_x = trajectory.tx[target_index]
#         target_y = trajectory.ty[target_index]
#     else:
#         target_x = trajectory.tx[-1]
#         target_y = trajectory.ty[-1]
#         target_index = len(trajectory.tx) - 1
#
#     distance_error = state.cal_distance(target_x, target_y)
#     direction_error = state.cal_direction(target_x, target_y)
#     sign = -1 if direction_error > 0 else 1
#     return sign * distance_error, target_index
#
# # 애니메이션 업데이트 함수
    # plt.cla()
    # 
    # # 에러 및 목표 인덱스 계산
    # cur_error, target_index = calculate_error(state, target_course, target_index)
    # total_error += cur_error ** 2
    # cur_input = controller.PD(cur_error)
    # state.update(None, cur_input)
    #
    # # 상태 기록 업데이트
    # state_list.append(frame * dt, state)
    # 
    # 
    # if target_index >= len(target_path_x) - 1:
    #     ani.event_source.stop()
    #     return
    #
    # # 플롯 갱신
    # plot_arrow(state.x, state.y, state.yaw)
    # plt.plot(target_path_x, target_path_y, "-r", label="course")
    # plt.plot(state_list.x, state_list.y, "-b", label="trajectory")
    # plt.plot(target_path_x[target_index], target_path_y[target_index], "xg", label="target")
    # plt.axis("equal")
    # plt.grid(True)
 # 예시로 사용할 업데이트 함수
def update(frame) -> Iterable[Artist]:
    ax.cla()

    """ Func method """
    # ax.plot(xp, yp)
    # ax.plot(xp, -yp)
    # ax.plot(xp[frame % len(xp)], yp[frame % len(xp)], 'ro') 

    """ Theta method """
    ax.plot(xp, yp, 'b')  # 원 그리기
    ax.plot(xp[frame], yp[frame], 'ro')  # 움직이는 점


    ax.set_xlim(-R - 1, R + 1)
    ax.set_ylim(-R - 1, R + 1)

    ax.set_aspect('equal', 'box')
    ax.set_title(f'Frame: {frame}')
    return ax.lines
    # 데이터를 그리거나 업데이트할 작업 수행
    return ax

# xp = np.linspace(-R, R, 100)
# yp = np.sqrt(R**2 - xp**2)

##################################################
R = 10
##################################################

theta = np.linspace(0, 2 * np.pi, 100)
xp = R * np.cos(theta)
yp = R * np.sin(theta)

frame_count = len(xp)
fig1, ax = plt.subplots(figsize=(7, 7))
ani = FuncAnimation(fig1, update, frames=frame_count, repeat=True, interval=100)
plt.show()


# ani.save('animation.mp4', writer='ffmpeg', fps=30)
