import rtde_control
import math
import numpy as np

ROBOT_ADDRESS = "192.168.0.10"

JOINT_VELOCITY     = 1.6
JOINT_ACCELERATION = 1

pi = math.pi

HOME = [0, -pi/2, 0, -pi/2, 0, 0]

#Set of configurations that realize the same EE pose
C  = np.array(( [1.0472, -1.2833, -0.7376, -2.6915, -1.5708,  3.1416],
                [1.0472, -1.9941,  0.7376,  2.8273, -1.5708,  3.1416],
                [1.0472, -1.5894, -0.5236,  0.5422,  1.5708,  0.0000],
                [1.0472, -2.0944,  0.5236,  0.0000,  1.5708,  0.0000],
                [2.7686, -1.0472, -0.5236,  3.1416, -1.5708,  1.4202],
                [2.7686, -1.5522,  0.5236,  2.5994, -1.5708,  1.4202],
                [2.7686, -1.1475, -0.7376,  0.3143,  1.5708, -1.7214],
                [2.7686, -1.8583,  0.7376, -0.4501,  1.5708, -1.7214]))

#Connect to the robot
rtde_c = rtde_control.RTDEControlInterface(ROBOT_ADDRESS)

#Move home
rtde_c.moveJ(HOME, JOINT_VELOCITY, JOINT_ACCELERATION)
#Move the robot in the 8 configurations that realize the pose
for i in range(0, C.shape[0]):
    rtde_c.moveJ(C[i, :], JOINT_VELOCITY, JOINT_ACCELERATION)