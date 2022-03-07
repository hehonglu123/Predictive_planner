#!/usr/bin/env python3
from RobotRaconteur.Client import *
import time
import numpy as np
import sys
sys.path.append('toolbox')
from sawyer_ik import inv as sawyer_inv
from abb_ik import inv as abb_inv

R_sawyer=np.array([[ 0., 0., -1. ],
 [ 0., -1.,  0.],
 [-1.,  0., 0.]])
R_UR=np.array([[-1,0,0],
			[0,0,-1],
			[0,-1,0]])
R_abb=np.array([[0,0,1],[0,1,0],[-1,0,0]])
R_staubli=np.array([[ -1, 0., 0 ],
 [ 0., 1,  0.],
 [0,  0., -1]])

robot = RRN.ConnectService('rr+tcp://localhost:58654?service=robot')        #Sawyer
robot3 = RRN.ConnectService('rr+tcp://localhost:58655?service=robot')       #ABB


robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = jog_mode



robot3.command_mode = halt_mode
time.sleep(0.1)
robot3.command_mode = jog_mode




# robot.jog_freespace(sawyer_inv([0.9,0.0,0.5],R_sawyer).reshape((7,1)), np.ones((7,)), False)
robot.jog_freespace([0,-0.6,0,0,0,0,0], np.ones((7,)), False)


p=abb_inv([0.66,0.5,0.6],R_abb).reshape((6,1))
q=[ 0.64833199, 0.99963736,-0.99677272,-0.        , 1.56793168, 0.64833199]
robot3.jog_freespace(p, np.ones((6,)), False)


