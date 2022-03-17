from multiprocessing import Process
import sys
from RobotRaconteur.Client import *
import yaml, time, traceback, threading, sys
import numpy as np
sys.path.append('toolbox')

from vel_emulate_sub import EmulatedVelocityControl


if __name__=='__main__':
    planner_inst=RRN.ConnectService('rr+tcp://localhost:25522/?service=Planner')

    sawyer_sub=RRN.SubscribeService('rr+tcp://localhost:58654?service=robot')
    state_w_sawyer = sawyer_sub.SubscribeWire("robot_state")
    cmd_w_sawyer=sawyer_sub.SubscribeWire('position_command')
    Sawyer=sawyer_sub.GetDefaultClientWait(1)

    ABB_sub=RRN.SubscribeService('rr+tcp://localhost:58655?service=robot')
    state_w_abb = ABB_sub.SubscribeWire("robot_state")
    cmd_w_abb=ABB_sub.SubscribeWire('position_command')
    ABB=ABB_sub.GetDefaultClientWait(1)

    robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", ABB)
    halt_mode = robot_const["RobotCommandMode"]["halt"]
    jog_mode = robot_const["RobotCommandMode"]["jog"]

    ###Command mode 
    ABB.command_mode = halt_mode
    time.sleep(0.1)
    ABB.command_mode = jog_mode

    start_joint_ABB=np.array([0.64833199, 0.99963736,-0.99677272,-0.        , 1.56793168, 0.64833199])
    ABB.jog_freespace(start_joint_ABB,np.ones(6),True)

    ###Command mode 
    Sawyer.command_mode = halt_mode
    time.sleep(0.1)
    Sawyer.command_mode = jog_mode

    start_joint_sawyer=np.array([0.5,-0.3,0,0,0,0,0])
    Sawyer.jog_freespace(start_joint_sawyer,np.ones(7),True)




    planner_inst.state_prop('sawyer',np.zeros(7*planner_inst.N_step))  ###update position
    planner_inst.state_prop('abb',np.zeros(6*planner_inst.N_step))  ###update position
