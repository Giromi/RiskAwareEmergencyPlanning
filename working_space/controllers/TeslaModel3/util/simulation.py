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
                         [150.0, 10.0],
                         [200.0, 0.0],
                        ])

llm_01 = np.array([ [35, -5],
                    [70, -10],
                    [95, 0],
                    [120, -10],
                    [160, 15],
                    [200, -10] ])
wooyeol_01 = np.array([[ 147, -18],
                       [ 143, -39],
                       [ 135, -57]])

one_collision = np.array([ [100.0, 0.0] ])

just_straight = [ np.array([100.0, 5.0]),
                  np.array([200.0, 5.0]) ]

def request_to_LLM():
    # return llm_01
    return collision_02

###############################################################################

def make_situation(driver, dt):
    tesla_state = TeslaState(driver, dt)

    tesla_state.update()
    init_x, init_y = tesla_state.x, tesla_state.y

    standard_speed = 1#TARGET_SPEED - 3 # [m/s]
    tesla_state.set_speed(TARGET_SPEED * 3.6) # [km/h]
    print(f'[Moral Machine] : {standard_speed * 3.6:.2f}[km/h] 속도를 감지하겠습니다.')
    while driver.step() != -1 and tesla_state.v <= standard_speed: # [m/s]
        tesla_state.update()
        # print(f'Target  speed (km/h): {tesla_state.get_target_seed() * 3.6}')
        print(f'current speed (km/h): {tesla_state.v * 3.6}')
        # print(f'current speed 2(km/h): {tesla_state.get_speed_km_h()}')
    print('[Moral Machine] : 이제부터, 이 핸들은 제껍니다.')
    distance = np.linalg.norm([tesla_state.x - init_x, tesla_state.y - init_y])
    driver.simulationSetMode(driver.SIMULATION_MODE_PAUSE)
    print(f'Accuration distance : {distance}')
    return tesla_state

###############################################################################


# def test_simulation(driver, dt):
#     tesla_state = TeslaState(driver, dt)
#
#     tesla_state.update()
#     while driver.step() != -1:
#         tesla_state.update()
#         print(f'Target  speed (km/h): {tesla_state.get_target_speed()}')
#         print(f'current speed 1(km/h): {tesla_state.get_speed() * 3.6}')
#         # print(f'current speed 2(km/h): {tesla_state.get_speed_km_h()}')
#     return tesla_state

