from debug import *


""" Webots """
from controller import Supervisor, Robot, Display, Camera   # 차후에 webots on/off할 때 필요
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
# from lib.rrt_planner import Node  #TODO

""" Util """
from util.operator import angle_mod, smooth_yaw
from util.map_maker import generate_grid_map
from util.simulation import request_to_LLM
from lib.convention import *
from lib.spline2d_planner import Spline2dPlanner


###############################################################################

def TEST_00():
    driver = Driver()
    old_at = driver.getTime()
    while driver.step() != -1:  
        now = driver.getTime()
        print(now, '>', now - old_at)
        old_at = driver.getTime()
''' # 시간 주기 확인
0.008임을 확인할 수 있다. 
출력은 [s] 단위이므로 
Webtos에서 설정은 [ms] 단위이다.
'''

def TEST_01():
    driver = Driver()
    dt = driver.getBasicTimeStep() / 1000 # [s] 늘려야할 수도 있음
    tesla_state = TeslaState(driver, dt)
    while True:
        tesla_state.update()
        print(tesla_state.get_time())
        if (tesla_state.x >= 5):
            break
''' # driver.step 역할 확인
driver.step() != -1 이 아니라 while True로 돌리면,
출력이 나오지 않고, 비정상적인 종료와 함께 한번에 몰아서 나온다.


'''

def TEST_02():
    driver = Driver()
    dt = driver.getBasicTimeStep() / 1000 # [s] 늘려야할 수도 있음
    tesla_state = TeslaState(driver, dt)

    while driver.step() != -1:
        tesla_state.update()
        print(tesla_state.get_time())
        if (tesla_state.x >= 5):
            break

''' # main 종료 확인
main문이 끝나도 시뮬레이션은 계속 작동한다.

'''

def TEST_02_1():
    driver = Driver()
    dt = driver.getBasicTimeStep() / 1000 # [s] 늘려야할 수도 있음
    tesla_state = TeslaState(driver, dt)

    driver.step()
    tesla_state.update()
    driver.step()
    tesla_state.set_speed(10)
    driver.step()
    driver.step()
    driver.step()
    driver.step()
    driver.step()
    driver.step()
    driver.step()
    driver.step()

def TEST_02_2():
    driver = Driver()
    dt = driver.getBasicTimeStep() / 1000 # [s] 늘려야할 수도 있음
    tesla_state = TeslaState(driver, dt)

    tesla_state.update()

'''
여기서 TEST_02에서 혼동이 올 수 있는 점은
main문이 끝나도 시뮬레이션이 작동하는 것처럼 보이기 때문에, 
TESET_02_1이 맞지만,
TEST_02_2처럼 해도 작동하는 것처럼 보여져서 조심해야 한다.
'''


def TEST_03_X(): # 실행시키지 말것(작업관리자 강제종료해야 함)
    MAX_TIME = 10
    driver = Driver()
    dt = driver.getBasicTimeStep() / 1000 # [s] 늘려야할 수도 있음
    tesla_state = TeslaState(driver, dt)
    while driver.step() != -1:
        points_collision = request_to_LLM()
        for cur_collision in points_collision:
            print(cur_collision)
            tesla_state.update()
            while MAX_TIME > tesla_state.get_time():
                print(tesla_state.get_time())
                if (tesla_state.x >= 5): # == check_goal
                    break

''' # 우리의 전체 로직 검증
정학히 한 스텝인 0.008초에 멈추면서,
webots 자체가 프로그램이 멈춰버린다.
'''

def TEST_04():
    driver = Driver()
    MAX_TIME = 1000 
    dt = driver.getBasicTimeStep() / 1000 # [s] 늘려야할 수도 있음
    tesla_state = TeslaState(driver, dt)
    points_collision = request_to_LLM()
    for cur_collision in points_collision:
        while driver.step() != -1 and MAX_TIME > tesla_state.get_time():
            print(f'current speed: {tesla_state.get_speed()}')
            tesla_state.update()
            print(f'now time     : {tesla_state.get_time()}')
            if (tesla_state.x >= cur_collision[X]): # == check_goal
                break

''' # 로직 수정 검증
while 안으로 step을 넣게 되면,
로직이 멈추지 않게 되고, 잘 진행한다.

'''

def is_simulation_pending(driver : Driver, tesla_state: TeslaState):
    return driver.step() != -1 and MAX_TIME > tesla_state.get_time()

def TEST_05():
    driver = Driver()
    dt = driver.getBasicTimeStep() / 1000 # [s] 늘려야할 수도 있음
    tesla_state = TeslaState(driver, dt)
    points_collision = request_to_LLM()
    for cur_collision in points_collision:
        print(f'TARGET(x, y, yaw) ⇒ {cur_collision}')
        while is_simulation_pending(driver, tesla_state):
            tesla_state.update(np.deg2rad(10))
            if (tesla_state.x >= cur_collision[X]): # == check_goal
                break
    tesla_state.set_speed(0)

''' # 가독성 증진 및 정지 기능 추가
is_simulation_end 함수를 만들어서, 가독성을 올릴 수 있고,
마지막에 속도를 0으로 만들어서, 정지할 수 있게 만들었다.
이 구조대로 로직을 작성하면 된다고 생각한다.
is_simulation_pending 은  TeslaState에 넣는 것이 효율적일 것 같다.
'''

def TEST_05():
    driver = Driver()
    dt = driver.getBasicTimeStep() / 1000 # [s] 늘려야할 수도 있음
    tesla_state = TeslaState(driver, dt)
    tesla_state.update()
    init_x, init_y = tesla_state.x, tesla_state.y
    points_collision = request_to_LLM()
    for cur_collision in points_collision:
        while is_simulation_pending(driver, tesla_state):
            tesla_state.update()
            print(f'current speed(m/s): {tesla_state.get_speed()}')
            print(f'current speed(km/h): {tesla_state.get_speed_km_h()}')
            if (tesla_state.v >= 100 / 3.6): # == check_goal
                print('speed limit')
                distance = np.linalg.norm([tesla_state.x - init_x, tesla_state.y - init_y])
                driver.simulationSetMode(driver.SIMULATION_MODE_PAUSE)
                print(f'need offset: {distance}')
                break

    # driver.getWidth
    #
    # width = wb_display_get_width(display);
    # height = wb_display_get_height(display);
    # wb_display_fill_rectangle(display,0,0,width,height);
    # wb_display_set_color(display,LIGHT_GREY);


def TEST_06(): # Spline Test
    # point_waypoints = request_to_LLM()

    x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    y = [0.7, -6, -5, -3.5, 0.0, 5.0, -2.0]
    yaw = np.deg2rad([0, 0, 0, 0, 0, 0, 0])

    point_waypoints = np.array([x, y, yaw]).T
    ds = 0.1  # [m] distance of each interpolated points

    print(f'{point_waypoints = }')
    plt.subplots(1)
    plt.plot(x, y, "xb", label="Data points")

    for (kind, label) in [("linear", "C0 (Linear spline)"),
                          ("quadratic", "C0 & C1 (Quadratic spline)"),
                          ("cubic", "C0 & C1 & C2 (Cubic spline)")]:
        spline2d_planner = Spline2dPlanner(point_waypoints, kind=kind)
        path = spline2d_planner.calculate()
        plt.plot(path[:, X], path[:, Y], label=label)

    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()
    plt.show()


def TEST_07():

    def generate_smooth_path(start_x, start_y, total_distance, num_points):
        # X 방향으로 일정한 간격으로 이동
        x = np.linspace(start_x, start_x + total_distance, num_points)
        
        # Y 방향으로 부드럽게 변화 (사인 곡선을 이용)
        y = 5 * np.sin(0.02 * x)  # 5m 진폭, 부드러운 변화
        
        # Yaw 계산 (경사각)
        yaw = np.arctan2(np.gradient(y), np.gradient(x))
        
        return np.vstack((x, y, yaw)).T

# 매끄러운 경로 생성
    smooth_path = generate_smooth_path(0, 0, 300, 100)  # 300m 거리, 100개 경로점

# np 형태로 출력
    np.set_printoptions(precision=2, suppress=True)
    print("Smooth Path as numpy array:")
    print("smooth_path = np.array([")
    for wp in smooth_path:
        print(f"    [{wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f}],")
    print("])")
    
