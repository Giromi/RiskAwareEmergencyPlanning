# 2024.11.17 Capdi
"""


class TeslaState(){
    self.current_position = ....
    self.current_orentation = ...

    
    def calculate_ref_trajectory(){
    }

    def update(ai, di){
    }
}

PathRRTPlanener 


class MPCPathTracker() {
    def __init__(.....){
        self.MPC = MPC()
    }
     yaw
    NU = 2  # a = [accel, steer]
    T = 5  # horizon length

    # mpc parameters
    R = np.diag([0.01, 0.01])  # input cost matrix  (값을 높게 설정하면 제어가 보수적으로 이루어져 차량이 안정적이지만 반응이 느려지고, 낮게 설정하면 민첩하게 반응하지만 불안정할 수 있음)
    Rd = np.diag([0.01, 1.0])  # input difference cost matrix (값을 높이면 차량이 급격한 조작을 피하고 부드럽게 반응하고, 낮추면 반대로 즉각적인 반응)
    Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix (값을 높게 설정한 상태에서는 주행 경로와의 오차가 줄어들며 경로 추적 성능이 향상되지만, 조향이 지나치게 민감)
    Qf = Q  # state final matrix (큰 값으로 설정하면 목표 지점에 도달할 때 더 정밀하게 조정하려고 하며, 작은 값은 목표에 덜 집착하고 일정한 패턴을 유지)
    GOAL_DIS = 20.0  # goal distance
    STOP_SPEED = 10 / 3.6  # stop speed
    MAX_TIME = 500.0  # max simulation time

    # iterative paramter
    MAX_ITER = 3  # Max iteration
    DU_TH = 0.1  # iteration finish param

    # target 
    TARGET_SPEED = 108 / 3.6  # [m/s] target speed # <<
    # TARGET_SPEED = 72  / 3.6  # [m/s] target speed # <<
    # TARGET_SPEED = 0  / 3.6  # [m/s] target speed # <<

    N_IND_SEARCH = 10  # Search index number

    DT = 0.1  # [s] time tick

    # Vehicle parameters TODO
    LENGTH = 4.724  # [m]
    WIDTH = 1.933  # [m]
    BACKTOWHEEL = 1.0  # [m]
    WHEEL_LEN = 0.3  # [m]  # 20 inch
    WHEEL_WIDTH = 0.2  # [m]
    TREAD = 1.584  # [m]
    WB = 2.875  # [m]

    MAX_STEER = np.deg2rad(35.0)  # maximum steering angle [rad]
    MAX_DSTEER = np.deg2rad(25.0)  # maximum steering speed [rad/s]
    MAX_SPEED = 261.0 / 3.6  # maximum speed [m/s]  # 261.0 km/h
    MIN_SPEED = -24.0 / 3.6  # minimum speed [m/s]
    MAX_ACCEL = 3.2  # maximum accel [m/ss]

}


def main(){    # <Main 문>
    while (step != -1){
    snake_case  : 변수, 함수, 파일명
    CamelCase   : 클래스
    pascalCase  : 사용 ㄴㄴ
    CAPITAL     :  상수

    driver = Driver()
    car_node = driver.getFromDef("TeslaModel3")
 
    #########################################################################
    DT = driver.getStep() # TODO endTime - startTime
    #########################################################################
    
    collision_points = LLM에게 요청(info.txt, current_pos)
    path_hanlder = PathHanlder(start, collision_points, DubinsPathPlanner)
    path_hanlder.calculate_path()

    cur_state = TeslaState( ... ) # x y yaw v t d a
    all_history = History( ... )

    all_history.append(cur_state)

    MPC = MPCPathTracker()

    odelta, oa = None, None

    cyaw = smooth_yaw(cyaw)


    for 
    while (1경유지 도착){
        ##### explain
        d_ref : reference sta
        x_ref : reference state =>target(정답) 
        x_cur : current   state
        (주의) cur_state.x : cur_state앞에 붙으면 x좌표
        #####

        x_ref, target_index, d_ref = cur_state.calculate_ref_trajectory(cx, cy, cyaw, ck, sp, dl, target_index)
        # x_ref, target_index, d_ref = calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, target_ind)
        
        x_cur = [cur_state.x, cur_state.x, cur_state.y, cur_state.v, cur_state_yaw], 
        # x0 = [state.x, state.y, state.v, state.yaw]  # current state

        oa, odelta, ox, oy, oyaw, ov = mpc.control(
            x_ref, x_cur,
            d_ref, 
            oa, odelta
        )
        # oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(
            xref, x0, dref, oa, odelta)

        d_index, a_index = 0.0, 0.0
        if odelta is not None:
            d_index, a_index = odelta[0], oa[0]
            cur_state.update(a_index, d_index)
            # state = update_state(state, a_index, d_index)

        time += DT
        # time = time + DT



        all_history.append(cur_state)

        # x.append(state.x)
        # y.append(state.y)
        # yaw.append(state.yaw)
        # v.append(state.v)
        # t.append(time)
        # d.append(di)
        # a.append(ai)

        # from uilts.operation import angle_mod
        driver.setSteeringAngle(d_i)

        if check_goal(state, goal, target_ind, len(cx)):
            print("Goal")
            break 
    }
}

if __name__ == '__main__':
    main()

""" # 밑에다가 코드로 변환하가ㅣ

"""
    # path_hanlder = PathHanlder(start, collision_points, RRT)   # 세팅하는 곳
    # rrt later
"""

