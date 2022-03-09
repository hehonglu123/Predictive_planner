import numpy as np
import yaml
filename="UR.yaml"
with open(filename,'r') as file:
	H=np.array(yaml.load(file)['H'])

####imposing orthonormal constraints
R=H[:2,:2]
T=np.transpose(np.array([H[:2,2]]))
u,s,vh=np.linalg.svd(R)
R=np.dot(u,vh)
H=np.concatenate((np.concatenate((R,T),axis=1),np.array([[0,0,1]])),axis=0)
print(H)
yaw=np.arcsin(H[0][1])

distance=np.matmul(-np.linalg.inv(R),np.array([[H[0][2]],[H[1][2]]]))
print(yaw)
print(distance)