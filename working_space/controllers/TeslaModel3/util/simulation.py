import numpy as np

collision_01 = np.array([ [30.0, 2.0, np.deg2rad(15)],
                          [60.0, -2.0, np.deg2rad(-15)],
                          [90.0, 2.0, np.deg2rad(20)],
                          [120.0, -10.0, np.deg2rad(-20)],
                          [150.0, 12.0, np.deg2rad(25)],
                          [180.0, -15.0, np.deg2rad(-30)],
                          [210.0, 10.0, np.deg2rad(10)],
                          [240.0, -5.0, np.deg2rad(-10)],
                          [270.0, 3.0, np.deg2rad(5)],
                          [300.0, 0.0, np.deg2rad(0)] ])

collision_02 = np.array([ np.array([3.0, 5.0, np.deg2rad(15)]),
                np.array([6.0, -3.0,np.deg2rad(-15)]),
                np.array([9.0, 8.0, np.deg2rad(20)]) ])


one_collision = [ np.array([100.0, 0.0, np.deg2rad(0)]) ]

just_straight = [ np.array([100.0, 5.0, np.deg2rad(0)]),
                  np.array([200.0, 5.0, np.deg2rad(0)]) ]

def request_to_LLM():
    return one_collision

def make_situation():
    driver = Driver()   # 차량, 건물 및 object의 객체
    tesla_state = TeslaState(driver, 0)

    tesla_state.update()
    init_x, init_y = tesla_state.x, tesla_state.y

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
    return driver, tesla_state

