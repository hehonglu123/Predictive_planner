import tesseract
import os
import re
import traceback
from tesseract_viewer import TesseractViewer
import numpy as np
import time
import sys
import yaml
sys.path.append('../../')
from gazebo_model_resource_locator import GazeboModelResourceLocator

with open("combined.urdf",'r') as f:
    combined_urdf = f.read()
with open("combined.srdf",'r') as f:
    combined_srdf = f.read()

pose=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],dtype=np.float64)


t = tesseract.Tesseract()

t.init(combined_urdf, combined_srdf, GazeboModelResourceLocator())

t_env = t.getEnvironment()

viewer = TesseractViewer()

viewer.update_environment(t_env, [0,0,0])

#link and joint names in urdf
Sawyer_joint_names=["right_j0","right_j1","right_j2","right_j3","right_j4","right_j5","right_j6"]
Sawyer_link_names=["right_l0","right_l1","right_l2","right_l3","right_l4","right_l5","right_l6","right_l1_2","right_l2_2","right_l4_2","right_hand"]
ABB_joint_names=['ABB1200_joint_1','ABB1200_joint_2','ABB1200_joint_3','ABB1200_joint_4','ABB1200_joint_5','ABB1200_joint_6']
ABB_link_names=['ABB1200_link_1','ABB1200_link_2','ABB1200_link_3','ABB1200_link_4','ABB1200_link_5','ABB1200_link_6']

robot_joint_list=[Sawyer_joint_names,ABB_joint_names]
num_robot=len(robot_joint_list)

viewer.start_serve_background()

for i in range(num_robot):
	t_env.setState(robot_joint_list[i], np.zeros(len(robot_joint_list[i])))



with open('../calibration/Sawyer.yaml') as file:
	H_Sawyer 	= np.array(yaml.load(file)['H'],dtype=np.float64)
with open('../calibration/ABB.yaml') as file:
	H_ABB 	= np.array(yaml.load(file)['H'],dtype=np.float64)



t_env.changeJointOrigin("sawyer_pose", H_Sawyer)
t_env.changeJointOrigin("abb_pose", H_ABB)
time.sleep(1)
viewer.update_environment(t_env, [0,0,0])

if sys.version_info[0] < 3:
    raw_input("press enter")
else:
    input("press enter")