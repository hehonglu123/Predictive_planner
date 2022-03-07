#!/usr/bin/env python3
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics import tesseract_geometry
from tesseract_robotics.tesseract_common import Isometry3d, CollisionMarginData
from tesseract_robotics import tesseract_collision
from tesseract_robotics_viewer import TesseractViewer
import os
import re
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import yaml
import numpy as np
import time
import traceback
import threading
import sys
sys.path.append('toolbox')
from gazebo_model_resource_locator import GazeboModelResourceLocator

#convert 4x4 H matrix to 3x3 H matrix and inverse for mapping obj to robot frame
def H42H3(H):
	H3=np.linalg.inv(H[:2,:2])
	H3=np.hstack((H3,-np.dot(H3,np.array([[H[0][-1]],[H[1][-1]]]))))
	H3=np.vstack((H3,np.array([0,0,1])))
	return H3

class Planner(object):
	def __init__(self):
		self._lock=threading.RLock()
		self._running=False

		###look ahead steps
		self.N_step=20
		#load calibration parameters
		with open('calibration/Sawyer.yaml') as file:
			H_Sawyer 	= np.array(yaml.load(file)['H'],dtype=np.float64)
		with open('calibration/ABB.yaml') as file:
			H_ABB 	= np.array(yaml.load(file)['H'],dtype=np.float64)
		self.H_Sawyer=H42H3(H_Sawyer)
		self.H_ABB=H42H3(H_ABB)

		#Connect to robot service
		# Sawyer= RRN.ConnectService('rr+tcp://localhost:58654?service=robot')
		# ABB= RRN.ConnectService('rr+tcp://localhost:58655?service=robot')
		# Sawyer_state=Sawyer.robot_state.Connect()
		# ABB_state=ABB.robot_state.Connect()

		#link and joint names in urdf
		Sawyer_joint_names=["right_j0","right_j1","right_j2","right_j3","right_j4","right_j5","right_j6"]
		Sawyer_link_names=["right_l0","right_l1","right_l2","right_l3","right_l4","right_l5","right_l6","right_l1_2","right_l2_2","right_l4_2","right_hand"]
		ABB_joint_names=['ABB1200_joint_1','ABB1200_joint_2','ABB1200_joint_3','ABB1200_joint_4','ABB1200_joint_5','ABB1200_joint_6']
		ABB_link_names=['ABB1200_link_1','ABB1200_link_2','ABB1200_link_3','ABB1200_link_4','ABB1200_link_5','ABB1200_link_6']

		
		self.robot_name_list=['sawyer','abb']
		self.robot_linkname_dict={'sawyer':Sawyer_link_names,'abb':ABB_link_names}
		self.robot_jointname_dict={'sawyer':Sawyer_joint_names,'abb':ABB_joint_names}
		# self.robot_state_dict=['sawyer':Sawyer_state,'abb':ABB_state]

		######tesseract environment setup:

		with open("urdf/combined.urdf",'r') as f:
			combined_urdf = f.read()
		with open("urdf/combined.srdf",'r') as f:
			combined_srdf = f.read()

		self.t_env= Environment()
		self.t_env.init(combined_urdf, combined_srdf, GazeboModelResourceLocator())
		self.scene_graph=self.t_env.getSceneGraph()

		#update robot poses based on calibration file
		self.scene_graph.changeJointOrigin("sawyer_pose", Isometry3d(H_Sawyer))
		self.scene_graph.changeJointOrigin("abb_pose", Isometry3d(H_ABB))

		contact_distance=0.5
		monitored_link_names = self.t_env.getLinkNames()
		self.manager = self.t_env.getDiscreteContactManager()
		self.manager.setActiveCollisionObjects(monitored_link_names)
		self.manager.setCollisionMarginData(CollisionMarginData(contact_distance))


		#######viewer setup, for urdf setup verification only#########################
		self.viewer = TesseractViewer()

		self.viewer.update_environment(self.t_env, [0,0,0])

		self.viewer.start_serve_background()

	def viewer_joints_update(self,robots_joint_dict):

		joint_names=[]
		joint_values=[]
		for key in robots_joint_dict:
			joint_names.extend(self.robot_jointname_dict[key])
			joint_values.extend(robots_joint_dict[key])

		self.viewer.update_joint_positions(joint_names, np.array(joint_values))

	def Sawyer_link(self,J2C):
		if J2C==7:
			return 1
		elif J2C==8:
			return 1
		elif J2C==9:
			return 4
		elif J2C==10:
			return 7
		else:
			return J2C


	def start(self):
		self._running=True
		self._checker = threading.Thread(target=self.distance_check_robot)
		self._checker.daemon = True
		self._checker.start()
	def close(self):
		self._running = False
		self._checker.join()

	def distance_check_all(self,robots_joint_dict):
		#return collision vector d_all and Joint to collision J2C_all (count from 0) #

		###update collision robot joint configuration
		joint_names=[]
		joint_values=[]
		for key in robots_joint_dict:
			joint_names.extend(self.robot_jointname_dict[key])
			joint_values.extend(robots_joint_dict[key])

		self.t_env.setState(joint_names, np.array(joint_values))

		env_state = self.t_env.getState()
		self.manager.setCollisionObjectsTransform(env_state.link_transforms)

		result = tesseract_collision.ContactResultMap()
		contacts = self.manager.contactTest(result,tesseract_collision.ContactRequest(tesseract_collision.ContactTestType_CLOSEST))
		result_vector = tesseract_collision.ContactResultVector()
		tesseract_collision.flattenResults(result,result_vector)

		distances=[]
		nearest_points=[]
		names=[]
		for c in result_vector:
			distances.append(c.distance)
			nearest_points.append([c.nearest_points[0],c.nearest_points[1]])
			names.append([c.link_names[0],c.link_names[1]])

		d_all={}	###collision vector: Closest_Pt_env - Closest_Pt
		J2C_all={}
		min_distance={}
		min_idx={}
		for robot_name in self.robot_name_list:
			d_all[robot_name]=np.zeros(3)
			J2C_all[robot_name]=0
			min_distance[robot_name]=9

		for i in range(len(distances)):
			print(names[i],distances[i])
		# 	for robot_name in self.robot_name_list:
		# 		###make sure only 1 of robot link in collision objects
		# 		if names[i][0] in self.robot_linkname_dict[robot_name] and names[i][1] not in self.robot_linkname_dict[robot_name]:
		# 			if min_distance[robot_name]>distances[i]:
		# 				d_all[robot_name]=nearest_points[i][1]-nearest_points[i][0]
		# 				J2C_all[robot_name]=self.robot_linkname_dict[robot_name].index(names[i][0])
		# 				min_distance[robot_name]=distances[i]
		# 				min_idx[robot_name]=i

		# 		if names[i][0] not in self.robot_linkname_dict[robot_name] and names[i][1] in self.robot_linkname_dict[robot_name]:
		# 			if min_distance[robot_name]>distances[i]:
		# 				d_all[robot_name]=nearest_points[i][0]-nearest_points[i][1]
		# 				J2C_all[robot_name]=self.robot_linkname_dict[robot_name].index(names[i][1])
		# 				min_distance[robot_name]=distances[i]
		# 				min_idx[robot_name]=i


		# return d_all,J2C_all

	def distack_check_all_Nstep(self):

		return d_all_N,J2C_all_N


def main():

	distance_inst=Planner()
	robots_joint_dict={'sawyer':np.array([0,-0.6,0,0,0,0,0]),'abb':np.array([ 0.14833199, 0.99963736,-0.99677272,-0.        , 1.56793168, 0.64833199])}
	distance_inst.viewer_joints_update(robots_joint_dict)
	distance_inst.distance_check_all(robots_joint_dict)

	

	input('press enter to quit')

if __name__ == '__main__':
	main()

# with RR.ServerNodeSetup("Distance_Service", 25522) as node_setup:
# 	#register service file and service
# 	RRN.RegisterServiceTypeFromFile("../../robdef/edu.rpi.robotics.distance")
# 	distance_inst=create_impl()				#create obj
# 	# distance_inst.start()
# 	RRN.RegisterService("Environment","edu.rpi.robotics.distance.env",distance_inst)
# 	print("distance service started")

# 	# time.sleep(1)	
# 	# print(distance_inst.distance_matrix.reshape(distance_inst.num_robot,distance_inst.num_robot))


# 	input("Press enter to quit")









