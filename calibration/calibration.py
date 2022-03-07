import numpy as np
import yaml
from RobotRaconteur.Client import *
def calibrate(obj,ref):	#list of [x_c,y_c] and [x_r,y_r]
	obj=np.hstack((obj,np.ones((len(obj),1))))
	H,residuals,rank,s=np.linalg.lstsq(obj,ref,rcond=-1)
	return np.vstack((np.transpose(H),[0,0,1]))
def calibrate_plug(obj,ref,cam_origin):
	A=np.array([[obj[0],-obj[1]],[obj[1],obj[0]]])
	b=np.transpose(ref-cam_origin)
	print(A)
	print(b)
	temp=np.linalg.solve(A,b)
	a=temp[0]
	b=temp[1]
	H=np.array([[a,-b,cam_origin[0]],[-b,a,cam_origin[1]],[0,0,1],])
	return H


inst=RRN.ConnectService('rr+tcp://128.113.224.144:52222/?service=SmartCam',"cats",{"password":RR.RobotRaconteurVarValue("cats111!","string")},None,None)

robot_name=raw_input("robot name: ")
if robot_name=="UR":
	robot=RRN.ConnectService('tcp://128.113.224.144:2355/URConnection/Universal_Robot',"cats",{"password":RR.RobotRaconteurVarValue("cats111!","string")},None,None)
	filename="UR.yaml"
elif robot_name=="Sawyer":
	robot = RRN.ConnectService('rr+tcp://127.0.0.1:58653?service=sawyer')
	filename="Sawyer.yaml"
elif robot_name=="ABB":
	robot=RRN.ConnectService('tcp://128.113.224.144:8884/SawyerJointServer/ABB',"cats",{"password":RR.RobotRaconteurVarValue("cats111!","string")},None,None)
	filename="ABB.yaml"
	pass

robot.enable()
pose = robot.robot_state.PeekInValue()[0].kin_chain_tcp['position']

obj=[]
ref=[]
##########accurate version
raw_input("please remove robot and place 3 objects")
while (len(inst.objects)<3):
	print("number of objects detected: ",len(inst.objects))
	raw_input("please move robot away and place 3 objects")
detected_obj=inst.objects
for i in range(3):
	obj.append([detected_obj[i].x/1000.,detected_obj[i].y/1000.])
	raw_input("please put robot endeffector on top of object: "+detected_obj[i].name)
	pose = robot.robot_state.PeekInValue()[0].kin_chain_tcp['position']
	ref.append([pose['x'][0],pose['y'][0]])
H=calibrate(obj,ref)

###############plug and play version
# raw_input("please put robot endeffector on top of camera origin: ")
# pose = robot.robot_state.PeekInValue()[0].kin_chain_tcp['position']
# cam_origin=[pose['x'][0],pose['y'][0]]
# raw_input("please put robot endeffector on top of object: "+detected_obj[0].name)
# pose = robot.robot_state.PeekInValue()[0].kin_chain_tcp['position']
# obj=np.array([detected_obj[0].x/1000.,detected_obj[0].y/1000.])
# ref=np.array([pose['x'][0],pose['y'][0]])
# H=calibrate_plug(obj,ref,cam_origin)

print(H)
#dump to yaml file
dict_file={'H':H.tolist()}
with open(filename,'w') as file:
	yaml.dump(dict_file,file)

