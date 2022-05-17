#!/usr/bin/env python3
from tesseract_robotics.tesseract_environment import Environment, ChangeJointOriginCommand
from tesseract_robotics import tesseract_geometry
from tesseract_robotics.tesseract_common import Isometry3d, CollisionMarginData
from tesseract_robotics import tesseract_collision
from tesseract_robotics_viewer import TesseractViewer

import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import yaml, time, traceback, threading, sys
import numpy as np
from qpsolvers import solve_qp
from scipy.optimize import fminbound

sys.path.append('toolbox')
from gazebo_model_resource_locator import GazeboModelResourceLocator
from robots_def import *

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

		###look ahead steps and timestep
		self.N_step=20
		self.ts=0.1

		#load calibration parameters
		with open('calibration/Sawyer.yaml') as file:
			H_Sawyer 	= np.array(yaml.load(file)['H'],dtype=np.float64)
		with open('calibration/ABB.yaml') as file:
			H_ABB 	= np.array(yaml.load(file)['H'],dtype=np.float64)
		self.H_Sawyer=H42H3(H_Sawyer)
		self.H_ABB=H42H3(H_ABB)

		self.transformation={'sawyer':H_Sawyer,'abb':H_ABB}

		################kinematics tools##################################################
		with open('config/sawyer_robot_default_config.yml') as file:
			sawyer_robot=yml2robdef(file)
		self.robot_toolbox={'sawyer':sawyer_robot,'abb':abb1200(R_tool=np.eye(3),p_tool=np.zeros(3))}

		#Connect to robot service
		sawyer_sub=RRN.SubscribeService('rr+tcp://localhost:58654?service=robot')
		sawyer_state = sawyer_sub.SubscribeWire("robot_state")
		sawyer_cmd_w=sawyer_sub.SubscribeWire('position_command')
		self.Sawyer=sawyer_sub.GetDefaultClientWait(1)

		abb_sub=RRN.SubscribeService('rr+tcp://localhost:58655?service=robot')
		abb_state = abb_sub.SubscribeWire("robot_state")
		abb_cmd_w=abb_sub.SubscribeWire('position_command')
		self.ABB=abb_sub.GetDefaultClientWait(1)

		self.robot_state={'sawyer':sawyer_state,'abb':abb_state}


		#link and joint names in urdf
		Sawyer_joint_names=["right_j0","right_j1","right_j2","right_j3","right_j4","right_j5","right_j6"]
		Sawyer_link_names=["right_l0","right_l1","right_l2","right_l3","right_l4","right_l5","right_l6","right_l1_2","right_l2_2","right_l4_2","right_hand"]
		ABB_joint_names=['ABB1200_joint_1','ABB1200_joint_2','ABB1200_joint_3','ABB1200_joint_4','ABB1200_joint_5','ABB1200_joint_6']
		ABB_link_names=['ABB1200_link_1','ABB1200_link_2','ABB1200_link_3','ABB1200_link_4','ABB1200_link_5','ABB1200_link_6']

		#Robot dictionaries, all reference by name
		self.robot_name_list=['sawyer','abb']
		self.robot_linkname={'sawyer':Sawyer_link_names,'abb':ABB_link_names}
		self.robot_jointname={'sawyer':Sawyer_joint_names,'abb':ABB_joint_names}
		

		######tesseract environment setup:
		with open("urdf/combined.urdf",'r') as f:
			combined_urdf = f.read()
		with open("urdf/combined.srdf",'r') as f:
			combined_srdf = f.read()

		self.t_env= Environment()
		self.t_env.init(combined_urdf, combined_srdf, GazeboModelResourceLocator())
		self.scene_graph=self.t_env.getSceneGraph()

		#update robot poses based on calibration file
		cmd1 = ChangeJointOriginCommand("sawyer_pose", Isometry3d(H_Sawyer))
		cmd2 = ChangeJointOriginCommand("abb_pose", Isometry3d(H_ABB))

		self.t_env.applyCommand(cmd1)
		self.t_env.applyCommand(cmd2)

		#Tesseract reports all GJK/EPA distance within contact_distance threshold
		contact_distance=0.2
		monitored_link_names = self.t_env.getLinkNames()
		self.manager = self.t_env.getDiscreteContactManager()
		self.manager.setActiveCollisionObjects(monitored_link_names)
		self.manager.setCollisionMarginData(CollisionMarginData(contact_distance))


		#######viewer setup, for URDF setup verification in browser @ localhost:8000/#########################
		# self.viewer = TesseractViewer()

		# self.viewer.update_environment(self.t_env, [0,0,0])

		# self.viewer.start_serve_background()


		##########N step planner predicted joint configs#################
		self.robot_N_step={'sawyer':np.zeros((self.N_step+1,7)),'abb':np.zeros((self.N_step+1,6))} #0 to N_step
		self.u_all={'sawyer':np.zeros(self.N_step*7),'abb':np.zeros(self.N_step*6)} #0 to N_step-1 control input qdot
		
	#######################################update joint angles in Tesseract Viewer###########################################
	def viewer_joints_update(self,robots_joint):
		joint_names=[]
		joint_values=[]
		for key in robots_joint:
			joint_names.extend(self.robot_jointname[key])
			joint_values.extend(robots_joint[key].tolist())

		self.viewer.update_joint_positions(joint_names, np.array(joint_values))

	def Sawyer_link(self,J2C):
		if J2C==7:
			return 1
		elif J2C==8:
			return 1
		elif J2C==9:
			return 4
		elif J2C==10:
			return 6
		else:
			return J2C


	def start(self):
		self._running=True
		self._checker = threading.Thread(target=self.distance_check_background)
		self._checker.daemon = True
		self._checker.start()
	def stop(self):
		self._running = False
		self._checker.join()

	################################single step distance checker##########################################################
	def distance_check_all(self,robots_joint):
		#return collision vector d_all and Joint to collision J2C_all (count from 0) #

		###update collision robot joint configuration
		joint_names=[]
		joint_values=[]
		for key in robots_joint:
			joint_names.extend(self.robot_jointname[key])
			joint_values.extend(robots_joint[key])

		self.t_env.setState(joint_names, np.array(joint_values))

		env_state = self.t_env.getState()
		self.manager.setCollisionObjectsTransform(env_state.link_transforms)

		result = tesseract_collision.ContactResultMap()
		contacts = self.manager.contactTest(result,tesseract_collision.ContactRequest(tesseract_collision.ContactTestType_ALL))
		result_vector = tesseract_collision.ContactResultVector()
		tesseract_collision.flattenResults(result,result_vector)

		distances=[]
		nearest_points=[]
		names=[]
		for c in result_vector:
			distances.append(c.distance)
			nearest_points.append([c.nearest_points[0].flatten(),c.nearest_points[1].flatten()])
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
			for robot_name in self.robot_name_list:
				###make sure only 1 of robot link in collision objects
				if names[i][0] in self.robot_linkname[robot_name] and names[i][1] not in self.robot_linkname[robot_name]:
					if min_distance[robot_name]>distances[i]:
						d_all[robot_name]=nearest_points[i][1]-nearest_points[i][0]
						if robot_name=='sawyer':
							J2C_all[robot_name]=self.Sawyer_link(self.robot_linkname[robot_name].index(names[i][0]))
						else:
							J2C_all[robot_name]=self.robot_linkname[robot_name].index(names[i][0])
						min_distance[robot_name]=distances[i]
						min_idx[robot_name]=i

				if names[i][0] not in self.robot_linkname[robot_name] and names[i][1] in self.robot_linkname[robot_name]:
					if min_distance[robot_name]>distances[i]:
						d_all[robot_name]=nearest_points[i][0]-nearest_points[i][1]
						if robot_name=='sawyer':
							J2C_all[robot_name]=self.Sawyer_link(self.robot_linkname[robot_name].index(names[i][1]))
						else:
							J2C_all[robot_name]=self.robot_linkname[robot_name].index(names[i][1])
						min_distance[robot_name]=distances[i]
						min_idx[robot_name]=i

		###convert collision vector to local frame
		for robot_name in self.robot_name_list:
			d_all[robot_name]=np.dot(self.transformation[robot_name][:-1,:-1],d_all[robot_name])

		# print(d_all,J2C_all,min_distance['sawyer'])
		return d_all,J2C_all,min_distance

	###############################################distance checking for future N step########################################################
	def distance_check_all_Nstep(self,robots_joint_N):
		d_all_N={}
		J2C_all_N={}
		min_distance_all_N={}
		for robot_name in self.robot_name_list:
			d_all_N[robot_name]=[]
			J2C_all_N[robot_name]=[]
			min_distance_all_N[robot_name]=[]
		for i in range(len(robots_joint_N[self.robot_name_list[0]])):
			#check collision at ith step
			robots_joint={}
			for robot_name in self.robot_name_list:
				robots_joint[robot_name]=robots_joint_N[robot_name][i]
			d_all,J2C_all,min_distance_all=self.distance_check_all(robots_joint)
			for robot_name in self.robot_name_list:
				d_all_N[robot_name].append(d_all[robot_name])
				J2C_all_N[robot_name].append(J2C_all[robot_name])
				min_distance_all_N[robot_name].append(min_distance_all[robot_name])

		# print(d_all_N,J2C_all_N)
		return d_all_N,J2C_all_N,min_distance_all_N

	##########################################background distance checker, running continuously, based on predicted N step future ##################################
	def distance_check_background(self):
		now=time.time()		###running @ 500~1000Hz
		###collision checker, 1 current + future N collision check
		while self._running:
			with self._lock:
				# print(1/(time.time()-now))
				# now=time.time()
				self.d_all_N,self.J2C_all_N,self.min_distance_all_N=self.distance_check_all_Nstep(self.robot_N_step)

	##########################################state propogation, get real time joint angle, propogate with N step u #############################################
	def state_prop(self,robot_name,u_all):
		###update current real time joint position
		self.robot_N_step[robot_name][0]=self.robot_state[robot_name].InValue.joint_position
		###update N_step control input
		self.u_all[robot_name]=u_all
		for i in range(1,self.N_step+1):
			self.robot_N_step[robot_name][i]=self.robot_N_step[robot_name][i-1]+self.ts*self.u_all[robot_name][(i-1)*len(self.robot_jointname[robot_name]):i*len(self.robot_jointname[robot_name])]

	#######################################barrier function for collision, push du based on signed collision distance########################################
	def barrier(self,x):
		a=1;b=5;e=0.05;l=1;
		return np.divide(a*b*(x-e),l+b*(x-e))

	###line search with qp
	def search_func(self,alpha,robot_name,qd,u_all,du_all):
		q_cur=self.robot_state[robot_name].InValue.joint_position
		qf=q_cur+np.sum(self.ts*(u_all+alpha*du_all).reshape((len(self.robot_jointname[robot_name]),-1)),axis=1)
		return np.linalg.norm(qf-qd)

	def d_col2q_col(self,d_col,jac):
		###convert collision vector from cartesian space to joint space
		H=np.dot(jac.T,jac)
		f=np.dot(d_col,jac)
		d_q=solve_qp(H,f)
		return d_q
	##############multi step planner for initial planning, take gradient Niter times#############
	def plan_initial(self,robot_name,qd,Niter):
		###iterate a few times before execution
		###initialize random control input toward desired direction
		q_cur=self.robot_state[robot_name].InValue.joint_position
		u_all=np.zeros(len(self.robot_jointname[robot_name])*self.N_step)
		self.state_prop(robot_name,u_all)

		for i in range(Niter):
			##############################################form collision constraint####################################
			try:
				Aineq=np.zeros((self.N_step,len(self.robot_jointname[robot_name])*self.N_step))
				bineq=np.zeros(self.N_step)
				time.sleep(0.01)###wait for thread updates q_all
				for k in range(1,self.N_step+1):
					if np.linalg.norm(self.d_all_N[robot_name][k])!=0:
						J2C=self.J2C_all_N[robot_name][k]												#get kth step joint to collision
						Jacobian2C=self.robot_toolbox[robot_name].jacobian(self.robot_N_step[robot_name][k])[:,:J2C+1]						#get the jacobian to that joint
						min_distance=self.min_distance_all_N[robot_name][k]
						print(robot_name,min_distance,'J2C',J2C,'step',k)
						d_col=self.d_all_N[robot_name][k]*np.sign(min_distance)
						d_q=np.dot(np.linalg.pinv(Jacobian2C[3:]),d_col)		##convert collision vector d to joint space
						d_q=d_q/np.linalg.norm(d_q)								##normalize collision vector d_q as direction only
						d_q=np.append(d_q,np.zeros(len(self.robot_jointname[robot_name])-len(d_q)))		##leave joints after J2C zero
						J_k=np.tile(np.eye(len(self.robot_jointname[robot_name])),(1,k))
						J_k=np.hstack((J_k,np.tile(np.zeros((len(self.robot_jointname[robot_name]),len(self.robot_jointname[robot_name]))),(1,self.N_step-k))))
						Aineq[k-1,:]=np.dot(d_q,J_k)
						bineq[k-1]=self.barrier(min_distance)					##barrier to push robot away from collision

			############################################q constraint################################################


			############################################qdot constraint################################################
				vel_limit=np.tile(self.robot_toolbox[robot_name].joint_vel_limit,(self.N_step,1)).flatten()/2
				if robot_name=='abb':
					vel_limit=np.tile(self.robot_toolbox[robot_name].joint_vel_limit,(self.N_step,1)).flatten()/10

			##############################################solve QP#####################################################
			
				eps=0.05
				J=np.tile(np.eye(len(self.robot_jointname[robot_name])),(1,self.N_step))
				Kp=1*np.eye(len(self.robot_jointname[robot_name]))
				H=np.dot(J.T,J)+eps*np.eye(self.N_step*len(self.robot_jointname[robot_name]))
				F=np.dot(J.T,np.dot(Kp,self.robot_N_step[robot_name][-1]-qd))
				du_all=solve_qp(H,F,G=Aineq,h=bineq,lb=-vel_limit-u_all, ub=vel_limit-u_all)
				#if no solution from QP
				if isinstance(du_all, type(None)):				
					print('QP no solution')
					bineq_new=np.clip(bineq,-0.5,np.inf)
					du_all=solve_qp(H,F,G=Aineq,h=bineq_new,lb=-vel_limit-u_all, ub=vel_limit-u_all)
					if isinstance(du_all, type(None)):				
						bineq_new=np.clip(bineq,0,np.inf)
						du_all=solve_qp(H,F,G=Aineq,h=bineq_new,lb=-vel_limit-u_all, ub=vel_limit-u_all)
				alpha=1
				# alpha=fminbound(self.search_func,0,1,args=(robot_name,qd,u_all,du_all,))
				# print(alpha)
				u_all_new=u_all+alpha*du_all
			except:
				traceback.print_exc()
			################update N_step properties##############################################
			self.state_prop(robot_name,u_all)
			self.u_all[robot_name]=u_all

		return u_all[:len(self.robot_jointname[robot_name])].tolist()

	##############multi step planner, return instantaneous q_dot for execution#############
	def plan(self,robot_name,qd):
		#running @ 40Hz
		# now=time.time()
		##############################u_all, trajectory N_step initialization#####################################
		if np.linalg.norm(self.u_all[robot_name])==0:
			###initialize random control input toward desired direction
			q_cur=self.robot_state[robot_name].InValue.joint_position
			u_temp=0.2*(qd-q_cur)/np.linalg.norm(qd-q_cur)
			u_all=np.tile(u_temp,(self.N_step,1)).flatten()
			self.state_prop(robot_name,u_all)
		else:
			###pick up from previous iteration
			u_all=np.append(self.u_all[robot_name][len(self.robot_jointname[robot_name]):],np.zeros(len(self.robot_jointname[robot_name])))
			self.state_prop(robot_name,u_all)

		time.sleep(0.01)
		##############################################form collision constraint####################################
		try:
			Aineq=np.zeros((self.N_step,len(self.robot_jointname[robot_name])*self.N_step))
			bineq=np.zeros(self.N_step)
			time.sleep(0.01)###wait for thread updates q_all
			for k in range(1,self.N_step+1):
				if np.linalg.norm(self.d_all_N[robot_name][k])!=0:
					J2C=self.J2C_all_N[robot_name][k]												#get kth step joint to collision
					Jacobian2C=self.robot_toolbox[robot_name].jacobian(self.robot_N_step[robot_name][k])[:,:J2C+1]						#get the jacobian to that joint
					min_distance=self.min_distance_all_N[robot_name][k]
					print(robot_name,min_distance,'J2C',J2C,'step',k)

					d_col=self.d_all_N[robot_name][k]*np.sign(min_distance)
					d_q=np.dot(np.linalg.pinv(Jacobian2C[3:]),d_col)		##convert collision vector d to joint space
					d_q=d_q/np.linalg.norm(d_q)								##normalize collision vector d_q as direction only
					d_q=np.append(d_q,np.zeros(len(self.robot_jointname[robot_name])-len(d_q)))		##leave joints after J2C zero
					print('collision q:',d_q)
					J_k=np.tile(np.eye(len(self.robot_jointname[robot_name])),(1,k))
					J_k=np.hstack((J_k,np.tile(np.zeros((len(self.robot_jointname[robot_name]),len(self.robot_jointname[robot_name]))),(1,self.N_step-k))))
					Aineq[k-1,:]=np.dot(d_q,J_k)							##J*du*d_q
					bineq[k-1]=self.barrier(min_distance)
					print('barrier',self.barrier(min_distance))				##barrier to push robot away from collision

		############################################q constraint################################################


		############################################qdot constraint################################################
			vel_limit=np.tile(self.robot_toolbox[robot_name].joint_vel_limit,(self.N_step,1)).flatten()/2
			if robot_name=='abb':
				vel_limit=np.tile(self.robot_toolbox[robot_name].joint_vel_limit,(self.N_step,1)).flatten()/10


		##############################################solve QP#####################################################
		
			eps=0.05
			J=np.tile(np.eye(len(self.robot_jointname[robot_name])),(1,self.N_step))
			Kp=1*np.eye(len(self.robot_jointname[robot_name]))
			H=np.dot(J.T,J)+eps*np.eye(self.N_step*len(self.robot_jointname[robot_name]))
			F=np.dot(J.T,np.dot(Kp,self.robot_N_step[robot_name][-1]-qd))
			du_all=solve_qp(H,F,G=Aineq,h=bineq,lb=-vel_limit-u_all, ub=vel_limit-u_all)
			#if no solution from QP
			if isinstance(du_all, type(None)):				
				print('QP no solution')
				bineq_new=np.clip(bineq,-0.5,np.inf)
				du_all=solve_qp(H,F,G=Aineq,h=bineq_new,lb=-vel_limit-u_all, ub=vel_limit-u_all)
				if isinstance(du_all, type(None)):				
					bineq_new=np.clip(bineq,0,np.inf)
					du_all=solve_qp(H,F,G=Aineq,h=bineq_new,lb=-vel_limit-u_all, ub=vel_limit-u_all)

			if np.linalg.norm(bineq)==0:
				alpha=fminbound(self.search_func,0.01,1,args=(robot_name,qd,u_all,du_all,))
				print('alpha:',alpha)
			else:
				alpha=1
			# print('alpha:',alpha)
			# print('du_all',du_all)
			u_all_new=u_all+alpha*du_all
		except:
			traceback.print_exc()
		################update N_step properties##############################################
		self.state_prop(robot_name,u_all_new)
		self.u_all[robot_name]=u_all_new

		# print(time.time()-now)
		return u_all_new[:len(self.robot_jointname[robot_name])].tolist()


def main():

	with RR.ServerNodeSetup("Planner_Node", 25522) as node_setup:

		#register service file and service
		RRN.RegisterServiceTypeFromFile("robdef/edu.rpi.robotics.planner")
		planner_inst=Planner()				#create obj
		###update current joint position
		planner_inst.state_prop('sawyer',np.zeros(planner_inst.N_step*7))
		planner_inst.state_prop('abb',np.zeros(planner_inst.N_step*6))
		planner_inst.start()
		RRN.RegisterService("Planner","edu.rpi.robotics.planner.planner",planner_inst)
		print("planner service started")

		input("Press enter to quit")
		#stop background checker
		planner_inst.stop()
	

if __name__ == '__main__':
	main()











