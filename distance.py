#!/usr/bin/env python
import tesseract
import os
import re
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import yaml
import numpy as np
import time
import traceback
import thread
import threading
import sys
sys.path.append('../')
from gazebo_model_resource_locator import GazeboModelResourceLocator

#convert 4x4 H matrix to 3x3 H matrix and inverse for mapping obj to robot frame
def H42H3(H):
	H3=np.linalg.inv(H[:2,:2])
	H3=np.hstack((H3,-np.dot(H3,np.array([[H[0][-1]],[H[1][-1]]]))))
	H3=np.vstack((H3,np.array([0,0,1])))
	return H3

class create_impl(object):
	def __init__(self):
		self._lock=threading.RLock()
		self._running=False
		#load calibration parameters
		with open('calibration/Sawyer.yaml') as file:
			H_Sawyer 	= np.array(yaml.load(file)['H'],dtype=np.float64)
		with open('calibration/ABB.yaml') as file:
			H_ABB 	= np.array(yaml.load(file)['H'],dtype=np.float64)
		self.H_Sawyer=H42H3(H_Sawyer)
		self.H_ABB=H42H3(H_ABB)
		self.L2C=['','']

		#form H into RR transformation struct
		transformation=RRN.GetStructureType("edu.rpi.robotics.distance.transformation")

		transformation1=transformation()
		transformation1.name="Sawyer"
		transformation1.row=len(self.H_Sawyer)
		transformation1.column=len(self.H_Sawyer[0])
		transformation1.H=np.float16(self.H_Sawyer).flatten().tolist()


		transformation2=transformation()
		transformation2.name="ABB"
		transformation2.row=len(self.H_ABB)
		transformation2.column=len(self.H_ABB[0])
		transformation2.H=np.float16(self.H_ABB).flatten().tolist()
		
		self.transformations=[transformation1,transformation2]

		#Connect to robot service
		Sawyer= RRN.ConnectService('rr+tcp://localhost:58654?service=robot')
		ABB= RRN.ConnectService('rr+tcp://localhost:58655?service=robot')
		Sawyer_state=Sawyer.robot_state.Connect()
		ABB_state=ABB.robot_state.Connect()

		#link and joint names in urdf
		Sawyer_joint_names=["right_j0","right_j1","right_j2","right_j3","right_j4","right_j5","right_j6"]
		Sawyer_link_names=["right_l0","right_l1","right_l2","right_l3","right_l4","right_l5","right_l6","right_l1_2","right_l2_2","right_l4_2","right_hand"]
		ABB_joint_names=['ABB1200_joint_1','ABB1200_joint_2','ABB1200_joint_3','ABB1200_joint_4','ABB1200_joint_5','ABB1200_joint_6']
		ABB_link_names=['ABB1200_base_link','ABB1200_link_1','ABB1200_link_2','ABB1200_link_3','ABB1200_link_4','ABB1200_link_5','ABB1200_link_6']

		self.robot_state_list=[Sawyer_state,ABB_state]
		self.robot_link_list=[Sawyer_link_names,ABB_link_names]
		self.robot_joint_list=[Sawyer_joint_names,ABB_joint_names]
		self.num_robot=len(self.robot_state_list)
		self.distance_matrix=-np.ones(self.num_robot*self.num_robot)

		######tesseract environment setup:

		with open("urdf/combined.urdf",'r') as f:
			combined_urdf = f.read()
		with open("urdf/combined.srdf",'r') as f:
			combined_srdf = f.read()
		t = tesseract.Tesseract()
		t.init(combined_urdf, combined_srdf, GazeboModelResourceLocator())
		self.t_env = t.getEnvironment()
		#update robot poses based on calibration file
		self.t_env.changeJointOrigin("sawyer_pose", H_Sawyer)
		self.t_env.changeJointOrigin("abb_pose", H_ABB)

		contact_distance=0.1
		monitored_link_names = self.t_env.getLinkNames()
		self.manager = self.t_env.getDiscreteContactManager()
		self.manager.setActiveCollisionObjects(monitored_link_names)
		self.manager.setContactDistanceThreshold(contact_distance)
	def Sawyer_link(self,J2C):
		if J2C==7:
			return 1
		elif J2C==8:
			return 2
		elif J2C==9:
			return 4
		elif J2C==10:
			return 7
		else:
			return J2C


	def start(self):
		self._running=True
		self._camera = threading.Thread(target=self.distance_check_robot)
		self._camera.daemon = True
		self._camera.start()
	def close(self):
		self._running = False
		self._camera.join()

	def distance_check_robot(self):
		while self._running:
			with self._lock:
				try:
					self.L2C=['','','','']
					#reset distance matrix
					distance_matrix=-np.ones((self.num_robot,self.num_robot))
					#update all robot joints
					for i in range(self.num_robot):
						robot_joints=self.robot_state_list[i].InValue.joint_position
						self.t_env.setState(self.robot_joint_list[i], robot_joints)
					#get distance check
					env_state = self.t_env.getCurrentState()
					self.manager.setCollisionObjectsTransform(env_state.link_transforms)
					contacts = self.manager.contactTest(2)
					contact_vector = tesseract.flattenResults(contacts)
					distances = np.array([c.distance for c in contact_vector])
					nearest_points=np.array([c.nearest_points for c in contact_vector])
					names = np.array([c.link_names for c in contact_vector])
					for i in range(self.num_robot):
						for j in range(i+1,self.num_robot):
							min_distance=1
							# min_distance_index=0
							for m in range(len(distances)):
								if 	(names[m][0] in self.robot_link_list[i] and names[m][1] in self.robot_link_list[j]) and distances[m]<min_distance:
									min_distance=distances[m]
									self.L2C[i]=names[m][0]
								elif(names[m][0] in self.robot_link_list[j] and names[m][1] in self.robot_link_list[i]) and distances[m]<min_distance:
									min_distance=distances[m]
									self.L2C[i]=names[m][1]

							#update distance matrix
							if min_distance!=1:
								distance_matrix[i][j]=min_distance
								distance_matrix[j][i]=min_distance
					self.distance_matrix=distance_matrix.flatten()
				except:
					traceback.print_exc()

	def distance_check(self,robot_idx):
		with self._lock:
			distance_report=RRN.GetStructureType("edu.rpi.robotics.distance.distance_report")
			distance_report1=distance_report()
			#update other robot's joints
			for i in range(self.num_robot):
				robot_joints=self.robot_state_list[i].InValue.joint_position
				self.t_env.setState(self.robot_joint_list[i], robot_joints)

			env_state = self.t_env.getCurrentState()
			self.manager.setCollisionObjectsTransform(env_state.link_transforms)
			contacts = self.manager.contactTest(2)

			contact_vector = tesseract.flattenResults(contacts)

			distances = np.array([c.distance for c in contact_vector])
			nearest_points=np.array([c.nearest_points for c in contact_vector])
			names = np.array([c.link_names for c in contact_vector])
			# nearest_index=np.argmin(distances)

			

			min_distance=9
			min_index=-1
			Closest_Pt=[0.,0.,0.]
			Closest_Pt_env=[0.,0.,0.]
			#initialize
			distance_report1.Closest_Pt=Closest_Pt
			distance_report1.Closest_Pt_env=Closest_Pt_env

			for i in range(len(distances)):

				#only 1 in 2 collision "objects"
				if (names[i][0] in self.robot_link_list[robot_idx] or names[i][1] in self.robot_link_list[robot_idx]) and distances[i]<min_distance and not (names[i][0] in self.robot_link_list[robot_idx] and names[i][1] in self.robot_link_list[robot_idx]):
					min_distance=distances[i]
					min_index=i


			J2C=0
			if (min_index!=-1):
				if names[min_index][0] in self.robot_link_list[robot_idx] and names[min_index][1] in self.robot_link_list[robot_idx]:
					stop=1
					print("stop")
				elif names[min_index][0] in self.robot_link_list[robot_idx]:
					J2C=self.robot_link_list[robot_idx].index(names[min_index][0])
					Closest_Pt=nearest_points[min_index][0]
					Closest_Pt_env=nearest_points[min_index][1]
					print(names[min_index])
					print(distances[min_index])
				elif names[min_index][1] in self.robot_link_list[robot_idx]:
					J2C=self.robot_link_list[robot_idx].index(names[min_index][1])
					Closest_Pt=nearest_points[min_index][1]
					Closest_Pt_env=nearest_points[min_index][0]
					print(names[min_index])
					print(distances[min_index])

				if robot_idx==0:
					J2C=self.Sawyer_link(J2C)

				distance_report1.Closest_Pt=np.float16(Closest_Pt).flatten().tolist()
				distance_report1.Closest_Pt_env=np.float16(Closest_Pt_env).flatten().tolist()
				distance_report1.min_distance=np.float16(distances[min_index])
				distance_report1.J2C=J2C	
				
				
				return distance_report1

			return distance_report1

with RR.ServerNodeSetup("Distance_Service", 25522) as node_setup:
	#register service file and service
	RRN.RegisterServiceTypeFromFile("../../robdef/edu.rpi.robotics.distance")
	distance_inst=create_impl()				#create obj
	# distance_inst.start()
	RRN.RegisterService("Environment","edu.rpi.robotics.distance.env",distance_inst)
	print("distance service started")

	# time.sleep(1)	
	# print(distance_inst.distance_matrix.reshape(distance_inst.num_robot,distance_inst.num_robot))


	input("Press enter to quit")









