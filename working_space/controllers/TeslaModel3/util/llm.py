import numpy as np

collision_01 = np.array([ np.array([30.0, 5.0, np.deg2rad(15)]),
                np.array([60.0, -3.0, np.deg2rad(-15)]),
                np.array([90.0, 8.0, np.deg2rad(20)]),
                np.array([120.0, -10.0, np.deg2rad(-20)]),
                np.array([150.0, 12.0, np.deg2rad(25)]),
                np.array([180.0, -15.0, np.deg2rad(-30)]),
                np.array([210.0, 10.0, np.deg2rad(10)]),
                np.array([240.0, -5.0, np.deg2rad(-10)]),
                np.array([270.0, 3.0, np.deg2rad(5)]),
                np.array([300.0, -8.0, np.deg2rad(-5)]),
                np.array([330.0, 15.0, np.deg2rad(30)]),
                np.array([360.0, -20.0, np.deg2rad(-25)]),
                np.array([390.0, 18.0, np.deg2rad(20)]),
                np.array([420.0, -12.0, np.deg2rad(-20)]),
                np.array([450.0, 10.0, np.deg2rad(15)]),
                np.array([480.0, -7.0, np.deg2rad(-10)]),
                np.array([510.0, 5.0, np.deg2rad(5)]),
                np.array([540.0, -3.0, np.deg2rad(-5)]),
                np.array([570.0, 2.0, np.deg2rad(3)]),
                np.array([600.0, 0.0, np.deg2rad(0)]) ])

collision_02 = np.array([ np.array([3.0, 5.0, np.deg2rad(15)]),
                np.array([6.0, -3.0, np.deg2rad(-15)]),
                np.array([9.0, 8.0, np.deg2rad(20)]) ])


one_collision = [ np.array([100.0, 0.0, np.deg2rad(0)]) ]

just_straight = [ np.array([100.0, 5.0, np.deg2rad(0)]),
                  np.array([200.0, 5.0, np.deg2rad(0)]) ]

def request_to_LLM():
    return collision_02
          
