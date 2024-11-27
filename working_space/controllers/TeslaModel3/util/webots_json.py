# TeslaModel3.py에 from util.webots_json import convert_json
#                  line 21 : json = convert_json()

from controller import Supervisor   
import json

def convert_json():
    ################################################################
    ##                 Convert dictionary to json                 ##
    ################################################################
    def convert_json(input : dict):         
        with open("data.json", "w") as f:   # 파일 저장소: main과 같은 위치
            json.dump(input, f, ensure_ascii=False, indent=4)

    # Supervisor 객체 생성 
    supervisor = Supervisor()

    # 환경에서 사용 가능한 모든 노드의 리스트 가져오기
    root_node = supervisor.getRoot()
    children_field = root_node.getField("children")

    building_names = []
    trafficlight_names = []
    streetlight_names = []
    tree_names = []
    car_names = []
    sidewalk_names = []
    human_names = []

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
                elif "car" in def_name:
                    car_names.append(def_name)
                elif "sidewalk_block" in def_name:
                    sidewalk_names.append(def_name)
                elif "human" in def_name:
                    human_names.append(def_name)
                else:
                    continue
            else:
                continue
        else:
            continue

    fixed_dict = {
        'building_dict' : {},
        'trafficlight_dict' : {},
        'streetlight_dict' : {},
        'tree_dict' : {},
        'car_dict' : {}, # Telsa 제외 다른 차량 (주차되어있는 차량, 빨간 불 멈춤)
        'sidewalk_dict' : {},
        'human_dict' :{}
    }

    ##########################< building >##########################
    print("Building names:", building_names)
    for name in building_names:
        node = supervisor.getFromDef(name)

        if "building_residential" in name:  size = [5.4, 21, 7.2]
        elif "building_cyberbotics" in name: size = [20, 47, 20]
        elif "building_commerial" in name: size = [22, 41.5, 22]
        elif "building_construction" in name: size = [16, 32, 16]
        elif "building_hollow" in name: size = [21, 31, 21]
        elif "building_hotel" in name: size = [14, 50, 14]
        else: size = [
            node.getField("corners").getMFVec2f(0)[0] * 2,  
            node.getField("floorHeight").getSFFloat() * node.getField("floorNumber").getSFInt32(),
            node.getField("corners").getMFVec2f(0)[1] * 2]

        print(size)
        fixed_dict['building_dict'][name] = {
            'pos' : node.getPosition(),
            'ori' : node.getOrientation(),
            'size' : size,
        } 
    if node is not None: print("bounding box :", fixed_dict['building_dict'])
    else: print("not found")

    #########################################
    ##            traffic light            ##
    #########################################
    print("trafficlight names:", trafficlight_names)
    for name in trafficlight_names:
        node = supervisor.getFromDef(name)
        fixed_dict['trafficlight_dict'][name] = {
            'pos' : node.getPosition(),
            'radius' : 0.1         # node.getField("radius").getSFFloat()    # TODO : 값을 못찾음
        }   
    if node is not None: print("bounding box :", fixed_dict['trafficlight_dict'])
    else: print("not found")

    #########################################
    ##            streetlight              ##
    #########################################
    print("streetlight names:", streetlight_names)
    for name in streetlight_names:
        node = supervisor.getFromDef(name)

        fixed_dict['streetlight_dict'][name] = {
            'pos' : node.getPosition(),
            'radius' : node.getField("radius").getSFFloat() / 10000   # 1000 -> 0.1m
        }   
    if node is not None: print("bounding box :", fixed_dict['streetlight_dict'])
    else: print("not found")

    ############################< tree >############################
    print("tree names:", tree_names)
    for name in tree_names:
        node = supervisor.getFromDef(name)
        fixed_dict['tree_dict'][name] = {
            'pos' : node.getPosition(),
            'radius' : node.getField("radius").getSFFloat() / 10   # cm -> m
        }   
    if node is not None: print("bounding box :", fixed_dict['tree_dict'])
    else: print("not found")

    ############################< car >#############################
    print("car names:", car_names)
    for name in car_names:
        if "bus" in name:  size = [9.73, 2.9, 2.64]
        elif "bmw" in name: size = [5.55, 1.44, 1.73]
        elif "CitroenCZero" in name: size = [3.7, 0.9, 1.6]
        elif "lincolnMKZ" in name: size = [5.2, 0.8, 2]
        elif "rangerover" in name: size = [5, 2.1, 2]
        elif "toyota" in name: [4.2, 1.4, 1.8]
        elif "bsnzs" in name: [7, 2.6, 2]
        # elif "Truck" in name: [14, 4.5, 3.2]
        # elif "bike" in name: [3.5, 1.5, 1.2]
        else: continue
    
        node = supervisor.getFromDef(name)
        fixed_dict['car_dict'][name] = {
            'pos' : node.getPosition(),
            'ori' : node.getOrientation(),
            'size' : size,          
        }   
    if node is not None: print("bounding box:", fixed_dict['car_dict'])
    else: print("not found")

    ##########################< sidewalk >##########################
    print("sidewalk names:", sidewalk_names)
    for name in sidewalk_names:
        node = supervisor.getFromDef(name)
        fixed_dict['sidewalk_dict'][name] = {
            'pos' : node.getPosition(),
            'ori' : node.getOrientation(),
            'size' : [node.getField("size").getSFVec2f()[0], 0.13, node.getField("size").getSFVec2f()[1]]
        }
    if node is not None: print("bounding box:", fixed_dict['sidewalk_dict'])
    else: print("not found")

    ###########################< human >############################
    print("human names:", human_names)
    for name in human_names:
        node = supervisor.getFromDef(name)
        fixed_dict['human_dict'][name] = {
            'pos' : node.getPosition(),
            'ori' : node.getOrientation(),
            'size' : node.getField("boundingObject").getSFNode().getField("radius").getSFFloat()
        }
    if node is not None: print("bounding box:", fixed_dict['human_dict'])
    else: print("not found")

    ################################################################
    ##                 Convert dictionary to json                 ##
    ################################################################
    convert_json(fixed_dict)
