# from vehicle import Driver # <- inherit from this class
# from controller import GPS # <- Not need
from controller import Supervisor   # 차후에 webots on/off할 때 필요
from vehicle import Driver
from get_information import parse_proto, get_value

import numpy as np
import json

# Convert dictionary to json
def convert_json(input : dict):         
    with open("data.json", "w") as f:   # 파일 저장소: TeslaModel3_controller_1029(box).py와 같은 위치
        json.dump(input, f, ensure_ascii=False, indent=4)

# Supervisor 객체 생성 
supervisor = Supervisor()
driver = Driver()   # 차량, 건물 및 object의 객체

# 차량 및 건물 노드 접근
car_node = driver.getFromDef("TeslaModel3")  # 'car'는 정확한 DEF 이름이어야 함

# 환경에서 사용 가능한 모든 노드의 리스트 가져오기
root_node = supervisor.getRoot()
children_field = root_node.getField("children")

building_names = []
trafficlight_names = []
streetlight_names = []
tree_names = []
car_names = []
# human_names = []
# road_names = []
sidewalk_names = []

for i in range(children_field.getCount()):
    node = children_field.getMFNode(i)
    if node is not None:
        def_name = node.getDef()
        if def_name:  # DEF 이름이 있는 경우
            if "building" in def_name:
                building_names.append(def_name)
            elif "traffic_light" in def_name:
                trafficlight_names.append(def_name)
            elif "street_light" in def_name:
                 streetlight_names.append(def_name)
            elif "tree(" in def_name:
                tree_names.append(def_name)                 
            elif "car(" in def_name:
                car_names.append(def_name)
            # # elif "road" in def_name:
            # #      road_names.append(def_name)
            elif "sidewalk_block" in def_name:
                sidewalk_names.append(def_name)
            else:
                continue
        else:
            continue
    else:
        continue

# # 각 이름 리스트 출력
# print("Road names:", road_names)
# print("Sidewalk names:", sidewalk_names)
# print("Traffic light names:", trafficlight_names)
# print("Street light names:", streetlight_names)
# print("Building names:", building_names)


# road_names = ["road(1)", "road(2)", "road(3)", "road(4), road_intersection(1)"]
# sidewalk_names = ["sidewalk_block(1)", "sidewalk_block(2)", "sidewalk_block(3)", "sidewalk_block(4)", "sidewalk_block(5)", "sidewalk_block(6)", "sidewalk_block(7)", "sidewalk_block(8)"]
# trafficlight_names = ["generic_traffic_light(1)", "generic_traffic_light(2)", "generic_traffic_light(3)", "generic_traffic_light(4)", "generic_traffic_light(5)", "generic_traffic_light(6)", "generic_traffic_light(7)", "generic_traffic_light(8)"]
# streetlight_names = ["street_light(1)", "street_light(2)", "street_light(3)", "street_light(4)", "street_light(5)", "street_light(6)", "street_light(7)", "street_light(8)", "street_light(9)", "street_light(10)", "street_light(11)", "street_light(12)", "street_light(13)", "street_light(14)", "street_light(15)", "street_light(16)", "street_light(17)", "street_light(18)", "street_light(19)", "street_light(20)", "street_light(21)", "street_light(22)", "street_light(23)", "street_light(24)", "street_light(25)", "street_light(26)", "street_light(27)", "street_light(28)", "street_light(29)", "street_light(30)", "street_light(31)", "street_light(32)", "street_light(33)", "street_light(34)", "street_light(35)", "street_light(36)"]
# building_names = ["commercial_building", "fast_food_restaurant", "museum", "hotel", "church", "residential_tower", "residential_tower(1)"]

# TODO : (우선순위 낮음) 사람이 직접 def 입력 안하는 방법

tesla_array = np.array([])



fixed_dict = {
    'building_dict' : {},
    'trafficlight_dict' : {},
    'streetlight_dict' : {},
    'tree_dict' : {},
    'sidewalk_dict' : {},
    'person_dict' :{},
    'car_dict' : {} # Telsa 제외 다른 차량 (주차되어있는 차량, 빨간 불 멈춤)
}

#     'building_dict' : [],
#     'trafficlight_dict' : [],
#     'streetlight_dict' : [],
#     'tree_dict' : [],
#     'sidewalk_dict' : [],
#     'person_dict' :[],
#     'car_dict' : [],
# }
#########################################
##             building                ##
#########################################
print("Building names:", building_names)
# building_dict = {}     # 노드를 저장할 리스트
# building_nodes = []     # 노드를 저장할 리스트
# building_position = []  # 위치를 저장할 리스트
building_sizes = [
    [22, 41.5, 22],     # comm(1)
    [5, 3.5, 5],        # fast
    [31.7, 17.3, 12.2], # museum
    [13.7, 24.8, 13.7], # hotel
    [15.5, 21, 10],     # church
    [5.4, 21, 7.2],     # residen(2)
    [57.4, 20.2, 14.4], # residen(1)
    [20, 13, 20],       # building(1)
    [22, 41.5, 22],     # comm(2)
    [12.2, 11, 29.1],   # residen(3)
    [20, 13, 20]        # building(2)
]
idx = 0
for name in building_names:
    node = driver.getFromDef(name)
    
    file_name = '../../protos/' + name + '.proto'
    # print(file_name)
    with open(file_name, "r") as file:
        proto_content = file.read()
    proto_dict = parse_proto(proto_content)

    # fixed_dict['building_dict'].append({
    #     'pos' : node.getPosition(),
    #     'ori' : node.getOrientation(),
    #     'size' : building_sizes[idx]   #get_value(proto_dict, 'size'),
    # })

    fixed_dict['building_dict'][name] = {
        'pos' : node.getPosition(),
        'ori' : node.getOrientation(),
        'size' : building_sizes[idx]   #get_value(proto_dict, 'size'),
    } 
    idx += 1
    # building_nodes.append(node)  # 노드 저장
    # building_position.append(position)

if node is not None:
    print("bounding box :", fixed_dict['building_dict'])
else:
    print("not found")

#########################################
##            traffic light            ##
#########################################
print("trafficlight names:", trafficlight_names)
# trafficlight_dict = {}
for name in trafficlight_names:
    node = driver.getFromDef(name)

    fixed_dict['trafficlight_dict'][name] = {
        'pos' : node.getPosition(),
        'radius' : 0.1         # node.getField("radius").getSFFloat()    # TODO : 값을 못찾음
    }   
if node is not None:
    print("bounding box :", fixed_dict['trafficlight_dict'])
else:
    print("not found")

#########################################
##            streetlight              ##
#########################################
print("streetlight names:", streetlight_names)
# tree_dict = {}
for name in streetlight_names:
    node = driver.getFromDef(name)

    fixed_dict['streetlight_dict'][name] = {
        'pos' : node.getPosition(),
        'radius' : node.getField("radius").getSFFloat() / 10000   # 1000 -> 0.1m
    }   
if node is not None:
    print("bounding box :", fixed_dict['streetlight_dict'])
else:
    print("not found")

#########################################
##                  tree               ##
#########################################
print("tree names:", tree_names)
# tree_dict = {}
for name in tree_names:
    node = driver.getFromDef(name)

    fixed_dict['tree_dict'][name] = {
        'pos' : node.getPosition(),
        'radius' : node.getField("radius").getSFFloat() / 10  # 1.5 -> 0.15m
    }   
if node is not None:
    print("bounding box :", fixed_dict['tree_dict'])
else:
    print("not found")

#########################################
##                  car                ##
#########################################
print("car names:", car_names)
car_dict = {}
car_sizes = [    # TODO
    [1.73, 1.44, 5.55],   # car(1)
    [1.6, 0.9, 3.7],      # car(2)
    [3.2, 4.5, 14],       # car(3)
    [1.2, 1.5, 3.5],      # car(4)
    [2, 0.8, 5.2],        # car(5)
    [2, 2.1, 5],          # car(6)
    [1.8, 1.4, 4.2],      # car(7)
    [2.64, 2.9, 9.73]     # car(8)
]
idx = 0
for name in car_names:
    node = driver.getFromDef(name)
    
    file_name = '../../protos/' + name + '.proto'
    print(file_name)
    with open(file_name, "r") as file:
        proto_content = file.read()
    proto_dict = parse_proto(proto_content)

    fixed_dict['car_dict'][name] = {
        'pos' : node.getPosition(),
        'ori' : node.getOrientation(),
        'size' : car_sizes[idx],          # get_value(proto_dict, 'size'),
    }
    idx += 1
if node is not None:
    print("bounding box:", fixed_dict['car_dict'])
else:
    print("not found")

#########################################
##               sidewalk              ##
#########################################
print("sidewalk names:", sidewalk_names)
sidewalk_dict = {}
for name in sidewalk_names:
    node = driver.getFromDef(name)

    # file_name = '../../protos/' + name + '.proto'
    # print(file_name)
    # with open(file_name, "r") as file:
    #     proto_content = file.read()
    # proto_dict = parse_proto(proto_content)

    fixed_dict['sidewalk_dict'][name] = {
        'pos' : node.getPosition(),
        'ori' : node.getOrientation(),
        'size' : node.getField("size").getSFVec3f()
    }
if node is not None:
    print("bounding box:", fixed_dict['sidewalk_dict'])
else:
    print("not found")




#########################################
##    Convert dictionary to json       ##
#########################################
convert_json(fixed_dict)


#for name in tree_names:
  #   fixed_dict['tree_dict'][name] = {


driver.setSteeringAngle(0)  # 차량 조향 각도
driver.setCruisingSpeed(20)   # 차량 속도


while driver.step() != -1:  
    print('\n\n-------- START -------')
    # print("Time: ", driver.getTime())  # 시뮬레이션 경과 시간(sec)
    # 차량 정보
    # print("\nSpeed : ", driver.getCurrentSpeed())              # 속력 (m/s)
    # print("\nPosition : ", car_node.getPosition())             # 위치 (x, y, z)
    # print("\nOrientation : ", car_node.getOrientation())       # 방향 matrix
    # print("\nTransformation matrix :\n", car_node.getPose())   # 방향, 위치 matrix
    # tesla_array = np.array(car_node.getPose())
    # print(type(car_node.getPose()))
    # print(type(tesla_array))
    # print(fixed_dict['building_dict']['museum'])


    print('-------- END --------')












































