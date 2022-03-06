#!/usr/bin/env python

from RobotRaconteur.Client import *
import math
import numpy as np
import time
import yaml

import sys
sys.path.append('../../toolbox/')
from general_robotics_toolbox import R2q	#convert R to quaternion

server=RRN.ConnectService('rr+tcp://localhost:11346/?service=GazeboServer')
w=server.get_worlds(str(server.world_names[0]))
pose_dtype = RRN.GetNamedArrayDType("com.robotraconteur.geometry.Pose", server)

def decomp(H):
	return R2q(H[:3,:3]),H[:3,3]

def initialize(robot_sdf,model_name,H):
	q,d=decomp(H)
	model_pose = np.zeros((1,), dtype = pose_dtype)
	model_pose["orientation"]['w'] = q[0]
	model_pose["orientation"]['x'] = q[1]
	model_pose["orientation"]['y'] = q[2]
	model_pose["orientation"]['z'] = q[3]
	model_pose["position"]['x']=d[0]
	model_pose["position"]['y']=d[1]
	model_pose["position"]['z']=d[2]
	w.insert_model(robot_sdf, model_name, model_pose)



#read sdf file
model_name="sawyer"
f = open('../models/'+model_name+'/model.sdf','r')
robot_sdf = f.read()
with open('calibration/Sawyer.yaml') as file:
	H = np.array(yaml.load(file)['H'],dtype=np.float64)

initialize(robot_sdf,model_name,H)
#read sdf file
model_name="abb"
f = open('../models/'+model_name+'/model.sdf','r')
robot_sdf = f.read()
with open('calibration/ABB.yaml') as file:
	H = np.array(yaml.load(file)['H'],dtype=np.float64)

initialize(robot_sdf,"abb",H)





