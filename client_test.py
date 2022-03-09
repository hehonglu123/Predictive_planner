
from RobotRaconteur.Client import *
import yaml, time, traceback, threading, sys
import numpy as np

###connect to all RR services
Sawyer= RRN.ConnectService('rr+tcp://localhost:58654?service=robot')
ABB= RRN.ConnectService('rr+tcp://localhost:58655?service=robot')
planner_inst=RRN.ConnectService('rr+tcp://localhost:25522/?service=Planner')

Sawyer_state=Sawyer.robot_state.Connect()
ABB_state=ABB.robot_state.Connect()
robot_state={'sawyer':Sawyer_state,'abb':ABB_state}

###RR constants
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", Sawyer)

halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
trajectory_mode = robot_const["RobotCommandMode"]["trajectory"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
velocity_mode = robot_const["RobotCommandMode"]["velocity_command"]

###Command mode 
Sawyer.command_mode = halt_mode
ABB.command_mode = halt_mode
time.sleep(0.1)
Sawyer.command_mode = jog_mode
ABB.command_mode = jog_mode

start_joint_abb=np.array([ 0.44833199, 0.99963736,-0.99677272,-0.        , 1.56793168, 0.64833199])
start_joint_sawyer=np.array([0,-0.6,0,0,0,0,0])
ABB.jog_freespace(start_joint_abb,np.ones(6),False)
Sawyer.jog_freespace(start_joint_sawyer,np.ones(7),True)

####initialize robot joints to a specific config
robots_joint={'sawyer':start_joint_sawyer,'abb':start_joint_abb}
planner_inst.viewer_joints_update(robots_joint)
planner_inst.state_prop_RR('sawyer',np.zeros(planner_inst.N_step*7))
planner_inst.state_prop_RR('abb',np.zeros(planner_inst.N_step*6))