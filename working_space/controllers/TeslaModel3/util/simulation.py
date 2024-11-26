import numpy as np
from vehicle import Driver
from lib.tesla_state import IdealState, TeslaState, History
from lib.convention import *

collision_01 = np.array([ [30.0, 2.0],
                          [60.0, -2.0],
                          [90.0, 2.0],
                          [120.0, -10.0],
                          [150.0, 12.0],
                          [180.0, -15.0],
                          [210.0, 10.0],
                          [240.0, -5.0],
                          [270.0, 3.0],
                          [300.0, 0.0] ])

collision_02 = np.array([ 
                         [100.0, 10.0],
                         [200.0, 5.0],
                         [300.0, 0.0],
                        ])



one_collision = [ np.array([100.0, 0.0]) ]

just_straight = [ np.array([100.0, 5.0]),
                  np.array([200.0, 5.0]) ]

def request_to_LLM():
    return collision_02

###############################################################################

def make_situation():
    driver = Driver()   # 차량, 건물 및 object의 객체
    tesla_state = TeslaState(driver, 0)

    tesla_state.update()
    dt = driver.getBasicTimeStep() / 1000
    init_x, init_y = tesla_state.x, tesla_state.y
    ideal_state = IdealState(dt, x=tesla_state.x,
                                 y=tesla_state.y,
                                 yaw=tesla_state.yaw, v=0.0)

    standard_speed = TARGET_SPEED * 0.95
    print(f'[Moral Machine] : {standard_speed * 3.6:.2f}[km/h] 속도를 감지하겠습니다.')
    tesla_state.set_speed(TARGET_SPEED)
    while driver.step() != -1 and tesla_state.v <= standard_speed: # [km/h]
        tesla_state.update()
        print(f'current speed 1(km/h): {tesla_state.get_speed() * 3.6}')
        print(f'current speed 2(km/h): {tesla_state.get_speed_km_h()}')
    print('[Moral Machine] : 이제부터, 이 핸들은 제껍니다.')
    distance = np.linalg.norm([tesla_state.x - init_x, tesla_state.y - init_y])
    driver.simulationSetMode(driver.SIMULATION_MODE_PAUSE)
    print(f'Accuration distance : {distance}')
    return driver, tesla_state, ideal_state

