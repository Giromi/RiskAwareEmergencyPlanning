# from vehicle import Driver # <- inherit from this class
# from controller import GPS # <- Not need
# from controller import Supervisor
from vehicle import Driver

# Supervisor 객체 생성
driver = Driver()

# 차량 및 건물 노드 접근
car_node = driver.getFromDef("TeslaModel3")  # 'car'는 정확한 DEF 이름이어야 함
building_node = driver.getFromDef("BUILDING")  # 'BUILDING'은 정확한 DEF 이름이어야 함

if car_node is None:
    print("Car node not found!")
    exit(1)

if building_node is None:
    print("Building node not found!")
    exit(1)

driver.setSteeringAngle(0.2)  # 차량 조향 각도
driver.setCruisingSpeed(20)   # 차량 속도


while driver.step() != -1:
    print(' \n \n-------- START -------')
    print("Time: ", driver.getTime())
    print(" \nSpeed : ", driver.getCurrentSpeed())
    print(" \nPosition : ", car_node.getPosition())
    print(" \nOrientation : ", car_node.getOrientation())
    print(" \nTransformation matrix :\n", car_node.getPose())
    print("\nBuilding position: ",  building_node.getPosition())

    print('-------- END --------')