from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D
from capdilib.convention import Waypoint
from capdilib.dubins_path_planner import DubinsPathPlanner
import matplotlib.pyplot as plt
from utils.plot import make_plot
from capdilib.path_handler import PathHanlder
from capdilib.convention import X, Y, YAW
from capdilib.controller import DiscretPIDController
from capdilib.target_course import TargetCourse
import numpy as np
import math
from capdilib.states import State

def init(car):
    car.set_offsets([[], []])  # 초기 위치 비우기
    car.set_UVC([], [])        # 초기 방향 비우기
    return car,

# Update function for animation
def update(car, frame, path_x, path_y, controller, state, target_course, target_ind):
    target_ind, _ = target_course.search_target_index(state)
    di = controller.steering_angle_control(state, path_x, path_y, target_ind)
    state.update(100 * 3.6, di)  # Set speed to 100 km/h
    car.set_data(state.x, state.y, state.yaw)
    car.set_offsets([state.x, state.y])
    car.set_UVC(np.cos(state.yaw), np.sin(state.yaw))  # yaw 방향 설정
    return car,




class DraggableLine:
    def __init__(self, ax, path):
        self.ax = ax
        self.path = path
        self.lines = []
        # 애니메이션용 '차량' 포인트 설정
        self.car = ax.quiver([], [], [], [], angles='xy', scale_units='xy', scale=1, color='r')
        self.state = State(x=self.path[0][X], y=self.path[0][Y], yaw=self.path[0][YAW])
        self.target_course = None  # Target course를 나중에 초기화


        for i in range(len(self.path) - 1):
            line, = self.ax.plot([path[i][X], path[i + 1][X]], [path[i][Y], path[i + 1][Y]], 
                                 marker='.', color='r', lw=2, linestyle='--')
            self.lines.append(line)
        self.path_handler = PathHanlder(path, DubinsPathPlanner)
        self.controller = DiscretPIDController(Kp=0.1, Ki=0.01, Kd=0.01, T=1)
        self.path_x, self.path_y, self.path_yaw = self.path_handler.calculate_path()
        make_plot(self.ax, self.path_x, self.path_y, self.path_yaw)

        self.target_course = TargetCourse(self.path_x, self.path_y)
        self.target_ind, _ = self.target_course.search_target_index(self.state)


        
        self.drag_start = None
        self.press = None
        self.background = None
        self.ax.figure.canvas.mpl_connect('button_press_event', self.on_press)
        self.ax.figure.canvas.mpl_connect('button_release_event', self.on_release)
        self.ax.figure.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.ax.figure.canvas.mpl_connect('scroll_event', self.on_scroll)  # 마우스 휠 이벤트 연결
        self.ax.figure.canvas.draw()

    def on_scroll(self, event):
        scale_factor = 1.1 if event.button == 'up' else 0.9  # 휠 위로 스크롤 시 확대, 아래로 스크롤 시 축소
        self.zoom(event, scale_factor)

    def zoom(self, event, scale_factor=1.1):
        # 현재 축의 한계를 가져옵니다.
        cur_xlim = self.ax.get_xlim()
        cur_ylim = self.ax.get_ylim()
        
        # 마우스의 위치를 기반으로 확대/축소의 중심을 설정합니다.
        xdata = event.xdata
        ydata = event.ydata

        if xdata is not None and ydata is not None:  # 마우스가 그래프 내에 있을 때만 확대/축소 실행
            # 확대/축소 비율을 계산합니다.
            new_xlim = [(xdata - (xdata - cur_xlim[0]) / scale_factor),
                        (xdata + (cur_xlim[1] - xdata) / scale_factor)]
            new_ylim = [(ydata - (ydata - cur_ylim[0]) / scale_factor),
                        (ydata + (cur_ylim[1] - ydata) / scale_factor)]
            
            # 새로운 축의 한계를 설정합니다.
            self.ax.set_xlim(new_xlim)
            self.ax.set_ylim(new_ylim)
            self.ax.figure.canvas.draw()



    def __find_min_length_index(self, event):
        line_x, line_y = self.lines[0].get_data()
        v = np.array([line_x[0], line_y[0]]) - np.array([event.xdata, event.ydata])
        min_length = np.linalg.norm(v, ord=2)
        min_index = 0
        for i in range(len(self.lines)):
            line_x, line_y = self.lines[i].get_data()
            v = np.array([line_x[1], line_y[1]]) - np.array([event.xdata, event.ydata])
            norm = np.linalg.norm(v, ord=2)
            if norm < min_length:
                min_length = norm
                min_index = i + 1
        return min_index


    def on_press(self, event):
        if event.button == 3:  # 우클릭 (우클릭은 'button' 값이 3)
            self.drag_start = (event.x, event.y)  # 드래그 시작 지점 저장
            return
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return

        min_index = self.__find_min_length_index(event)
        self.press = (min_index, event.xdata, event.ydata)

    def on_motion(self, event):
        if self.drag_start and event.button == 3:  # 우클릭 상태에서만 실행
            dx = event.x - self.drag_start[0]
            dy = event.y - self.drag_start[1]

            cur_xlim = self.ax.get_xlim()
            cur_ylim = self.ax.get_ylim()
            offset = 0.1

            # 마우스 움직임에 따라 축 이동
            new_xlim = [cur_xlim[0] - dx * offset, cur_xlim[1] - dx * offset]
            new_ylim = [cur_ylim[0] - dy * offset, cur_ylim[1] - dy * offset]

            self.ax.set_xlim(new_xlim)
            self.ax.set_ylim(new_ylim)
            self.ax.figure.canvas.draw()

            # 드래그 시작 지점을 업데이트
            self.drag_start = (event.x, event.y)
            return

        #
        if self.press is None or event.inaxes != self.ax:
            return

        min_index, xpress, ypress = self.press

        print('click  : ', min_index)
        print('xpress : ', xpress)
        print('ypress : ', ypress)

       # 현재 축의 한계를 저장
        cur_xlim = self.ax.get_xlim()
        cur_ylim = self.ax.get_ylim()
        self.ax.cla()

        self.path[min_index][X], self.path[min_index][Y] = event.xdata, event.ydata
        # for i in range(len(self.lines) + 1):
        #     if i != min_index:
        #         continue
        #     if i != 0: # handling the first line
        #         prev_line_x, prev_line_y = self.lines[i - 1].get_data()
        #         # self.lines[i - 1].set_data([prev_line_x[0], event.xdata], [prev_line_y[0], event.ydata])
        #         self.ax.plot([prev_line_x[0], event.xdata], [prev_line_y[0], event.ydata], 'r--')
        #
        #     if i != len(self.lines): # handling the last line 
        #         next_line_x, next_line_y = self.lines[i].get_data()
        #         self.ax.plot([event.xdata, next_line_x[1]], [event.ydata, next_line_y[1]], 'r--')
                # self.lines[i].set_data([event.xdata, next_line_x[1]], [event.ydata, next_line_y[1]])

        for i in range(len(self.lines)):
            self.ax.plot([self.path[i][X], self.path[i + 1][X]], [self.path[i][Y], self.path[i + 1][Y]], 
                                 marker='.', color='r', lw=2, linestyle='--')

        # Update the path waypoints
        # self.path_handler.path_planner, self.path_handler.path[0][Y] = new_line_x[0], new_line_y[0]
        # self.path_handler.path[1][X], self.path_handler.path[1][Y] = new_line_x[1], new_line_y[1]
        self.path_x, self.path_y, self.path_yaw = self.path_handler.calculate_path()
        make_plot(self.ax, self.path_x, self.path_y, self.path_yaw)
        self.ax.set_xlim(cur_xlim)
        self.ax.set_ylim(cur_ylim)
        plt.grid(True)
        self.ax.figure.canvas.draw()

    def on_release(self, event):
        if event.button == 3:  # 우클릭 해제
            self.drag_start = None
            return

        self.press = None

        # 매번 경로가 업데이트될 때 `target_course`를 다시 설정합니다.
        self.target_course = TargetCourse(self.path_x, self.path_y)
        self.target_ind, _ = self.target_course.search_target_index(self.state)
        anim = FuncAnimation(
            self.ax.figure, update, frames=len(self.path_x),
            init_func=lambda: init(self.car),
            fargs=(self.car, self.path_x, self.path_y, self.controller, 
                   self.state, self.target_course, self.target_ind),
            blit=True, interval=50, repeat=True
        )

        plt.show()
        self.ax.figure.canvas.draw()
