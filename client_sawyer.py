
from RobotRaconteur.Client import *
import yaml, time, traceback, threading, sys
import numpy as np
sys.path.append('toolbox')

from vel_emulate_sub import EmulatedVelocityControl

###connect to all RR services
planner_inst=RRN.ConnectService('rr+tcp://localhost:25522/?service=Planner')

sawyer_sub=RRN.SubscribeService('rr+tcp://localhost:58654?service=robot')
state_w = sawyer_sub.SubscribeWire("robot_state")
cmd_w=sawyer_sub.SubscribeWire('position_command')
Sawyer=sawyer_sub.GetDefaultClientWait(1)

vel_ctrl = EmulatedVelocityControl(Sawyer,state_w, cmd_w)

###RR constants
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", Sawyer)

halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]
trajectory_mode = robot_const["RobotCommandMode"]["trajectory"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
velocity_mode = robot_const["RobotCommandMode"]["velocity_command"]

###Command mode 
Sawyer.command_mode = halt_mode
time.sleep(0.1)
Sawyer.command_mode = jog_mode

start_joint_sawyer=np.array([0.5,-0.3,0,0,0,0,0])
Sawyer.jog_freespace(start_joint_sawyer,np.ones(7),True)

###Command mode 
Sawyer.command_mode = halt_mode
time.sleep(0.1)
Sawyer.command_mode = position_mode


vel_ctrl.enable_velocity_mode()
qd=np.array([-2,-0.6,0,0,0,0,0])
planner_inst.plan_initial('sawyer',qd,50)
while np.linalg.norm(state_w.InValue.joint_position-qd)>0.1:
	qdot=planner_inst.plan('sawyer',qd)
	print(qdot)
	now=time.time()
	
	while time.time()-now<planner_inst.ts:
		vel_ctrl.set_velocity_command(qdot)


	# qd_temp=state_w.InValue.joint_position+planner_inst.ts*qdot
	# while np.linalg.norm(state_w.InValue.joint_position-qd_temp)>0.01:
	# 	vel_ctrl.set_velocity_command(qdot)
	# 	if time.time()-now>planner_inst.ts:
	# 		break

vel_ctrl.set_velocity_command(np.zeros((7,)))
vel_ctrl.disable_velocity_mode()
