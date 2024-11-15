from vehicle import Driver
import time    # while문 Hz 추출
from multiprocessing import Process, Value
import math

delta_shared = Value('d', 0.0)

# `mpc_dubins.py`의 main 함수를 멀티프로세싱으로 실행하여 delta 값을 공유 변수에 업데이트
def run_mpc():
    subprocess.run(["python", "../../path_tracker/MPC/mpc_dubins.py"])  # 파일 경로에 맞게 수정

# Supervisor 객체 생성 
driver = Driver()   # 차량, 건물 및 object의 객체
car_node = driver.getFromDef("TeslaModel3")  # 'car'는 정확한 DEF 이름이어야 함

# `mpc_dubins.py`의 시뮬레이션을 별도 프로세스로 실행
mpc_process = Process(target=run_mpc)
mpc_process.start()

while driver.step() != -1:  
    print('\n\n-------- START -------')
    # time step 확인
    # current_time = time.time()
    # elapsed_time = current_time - previous_time  # 루프 하나에 걸린 시간 (초)
    # previous_time = current_time
    # hz = 1 / elapsed_time if elapsed_time > 0 else float('inf')  # 주기에서 Hz 계산
    # print(f"Time: {driver.getTime():.10f} seconds")  # 시뮬레이션 경과 시간(sec)
    # print(f"Loop frequency: {hz:.2f} Hz")  # Hz 출력

    # 공유 변수에서 yaw 값을 실시간으로 가져와 Webots 조향 각도로 설정
    current_delta = delta_shared.value
    print('current_delta :', current_delta)
    driver.setSteeringAngle(current_delta)  # 조향 각도 설정
    driver.setCruisingSpeed(108)  # 속도 설정

    # 차량 정보
    print("\nSpeed : ", driver.getCurrentSpeed())              # 속력 (m/s)
    print("\nPosition : ", car_node.getPosition())             # 위치 (x, y, z)
    print("\nOrientation : ", car_node.getOrientation())       # 방향 matrix
    print('-------- END --------')
    time.sleep(0.01)  # 10ms 대기 (필요에 따라 조정 가능)
    
 # Webots 시뮬레이션 종료 시 MPC 프로세스 종료
mpc_process.terminate()
mpc_process.join()