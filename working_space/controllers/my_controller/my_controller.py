"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor


# from vehicle import Driver # <- inherit from this class
# from controller import GPS # <- Not need
import math 
from vehicle import Car
# from controller import Supervisor

car = Car()
# supervisor = Supervisor()
car_node = car.getFromDef("car")
# car_node = supervisor.getFromDef("car")
if car_node is None:
    print("Node not found")
    exit(1)

car.setSteeringAngle(0.2)
car.setCruisingSpeed(20)

#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# print("Device", car.devices)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while car.step() != -1:
    print(' \n  \n-------- START --------')
    print("Time : ", car.getTime())
    print(" \nSpeed : ", car.getCurrentSpeed())
    print(" \nPosition : ", car_node.getPosition())
    print(" \nOrientation : ", car_node.getOrientation())
    print(' \nTransformation matrix :\n', car_node.getPose())
    print('-------- END --------')



# while supervisor.step() != -1:
#     print("time: ", supervisor.getTime())
#     print("position: ", car_node.getPosition())

    # print("position: ", driver.getPosition())
    # print("angle: ", driver.getCurrentAngle())
    # print("distance: ", driver.getDistanceAt(0))
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.
