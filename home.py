#!/usr/bin/env python3
from RobotRaconteur.Client import *
import time
import numpy as np
import sys

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




robot.jog_freespace([0,-0.6,0,0,0,0,0], np.ones((7,)), False)


q=[ 0.64833199, 0.99963736,-0.99677272,-0.        , 1.56793168, 0.64833199]
robot3.jog_freespace(q, np.ones((6,)), False)


