
from RobotRaconteur.Client import *
import yaml, time, traceback, threading, sys
import numpy as np
sys.path.append('toolbox')

from vel_emulate_sub import EmulatedVelocityControl

###connect to all RR services
planner_inst=RRN.ConnectService('rr+tcp://localhost:25522/?service=Planner')

ABB_sub=RRN.SubscribeService('rr+tcp://localhost:58655?service=robot')
state_w = ABB_sub.SubscribeWire("robot_state")
cmd_w=ABB_sub.SubscribeWire('position_command')
ABB=ABB_sub.GetDefaultClientWait(1)

vel_ctrl = EmulatedVelocityControl(ABB,state_w, cmd_w)

###RR constants
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", ABB)

halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
trajectory_mode = robot_const["RobotCommandMode"]["trajectory"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
velocity_mode = robot_const["RobotCommandMode"]["velocity_command"]

###Command mode 
ABB.command_mode = halt_mode
time.sleep(0.1)
ABB.command_mode = jog_mode

start_joint_ABB=np.array([0.64833199, 0.99963736,-0.99677272,-0.        , 1.56793168, 0.64833199])
ABB.jog_freespace(start_joint_ABB,np.ones(6),True)
planner_inst.state_prop('abb',np.zeros(6*planner_inst.N_step))	###update position

###Command mode 
ABB.command_mode = halt_mode
time.sleep(0.1)
ABB.command_mode = position_mode


vel_ctrl.enable_velocity_mode()
qd=np.array([-0.64833199, 0.99963736,-0.99677272,-0.        , 1.56793168, 0.64833199])
planner_inst.plan_initial('abb',qd,50)
while np.linalg.norm(state_w.InValue.joint_position-qd)>0.1:
	qdot=planner_inst.plan('abb',qd)
	print(qdot)
	now=time.time()
	while time.time()-now<planner_inst.ts:
		vel_ctrl.set_velocity_command(qdot)
	# vel_ctrl.set_velocity_command(np.zeros(6))

vel_ctrl.set_velocity_command(np.zeros((6,)))
vel_ctrl.disable_velocity_mode()
