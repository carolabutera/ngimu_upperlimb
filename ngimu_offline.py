#!/usr/bin/python3
import matrix_op
import numpy as np
import math
from numpy.linalg import norm
import csv
   

# Function that computes the relative angle between two vectors:
def relative_angle(v1,v2):
    angle_rel = math.atan2(norm(np.cross(v1,v2),1),(np.dot(v1,np.transpose(v2))))
    return angle_rel

arm =1 #TIAGo arm


z_onto_xy = np.matrix([[0, 0, 0]])
vec = [0, 0, 0]


#per linee da a .... 
with open("221124_NGIMU_TIAGo_data/synchro_tiago_imu.csv", 'r') as file:
  csvreader = csv.reader(file)
  for index, row in enumerate(csvreader):
      if  index==2: 
        TO_90=np.matrix([[row[1],row[2],row[3]],[row[4],row[5],row[6]],[row[7],row[8],row[9]]])
print(TO_90)
      


sumTO_npose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
sumUA_npose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
sumFA_npose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
sumTO_tpose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
sumUA_tpose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
sumFA_tpose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
n=0
m=0
i=0

sumTO_npose=sumTO_npose+TO_g  #to store all the matrix in the n-pose
sumUA_npose=sumUA_npose+UA_g          
sumFA_npose=sumFA_npose+FA_g
n=n+1

sumUA_tpose=sumUA_tpose+UA_g
sumFA_tpose=sumFA_tpose+FA_g
m=m+1


TO_npose=sumTO_npose/n #mean of the matrix in the n-pose
UA_npose=sumUA_npose/n
FA_npose=sumFA_npose/n
#NB: n-poses are expressed wrt the global reference frame!

UA_tpose=sumUA_tpose/m #mean of the matrix in the t-pose
FA_tpose=sumFA_tpose/m
#NB: t-poses are expressed wrt the global reference frame!

#rotation matrix around the global axis to go from the n-pose to the t-pose (is the t-pose calibrated wrt to n-pose)
#theoretically it is np.matmul(TO_tpose, TO_npose.T) multiplied by an identity matrix which is the n_pose calibrated to itself
UA_tpose_calib=np.matmul(UA_tpose, UA_npose.T)
FA_tpose_calib=np.matmul(FA_tpose, FA_npose.T)

#alpha=angle between global y-axis and calibrated z-axis during t-pose
alpha=relative_angle(-arm*UA_tpose_calib[:,2].T,[0,1,0]) #we can average values from FA and UA? 

#theta=angle between global x-axis and calibrated z-axis during t-pose
if alpha < math.pi/2:
    theta=relative_angle(-arm*UA_tpose_calib[:,2].T, [1,0,0])

else:
    theta=2*math.pi-relative_angle(-arm*UA_tpose_calib[:,2].T, [1,0,0])

#matrix bewteen n-pose and body reference frame
TO_calib=np.matmul(matrix_op.rotZ(theta).T, TO_npose)
UA_calib=np.matmul(matrix_op.rotZ(theta).T, UA_npose)
FA_calib=np.matmul(matrix_op.rotZ(theta).T, FA_npose)



                
TO_b=np.matmul(matrix_op.rotZ(theta).T,TO_g)#Tranform to place y-axis perpendicular to the torso                
UA_b=np.matmul(matrix_op.rotZ(theta).T,UA_g)
FA_b=np.matmul(matrix_op.rotZ(theta).T,FA_g)

#Calibrated matrix (wrt to NPOSE initial position) expressed wrt to BODY reference frame
TO=np.matmul(TO_b, TO_calib.T)  
UA=np.matmul(UA_b, UA_calib.T)
FA=np.matmul(FA_b, FA_calib.T)

#NB: BODY reference frame:  y-axis perpendicular to torso, x-axis pointing to the right of the body and z-axis upward. 


# POE
#Method 1 to evaluate projection:
z_onto_y=np.dot(TO[:,1].T, UA[:,2], out=None) 
z_onto_x=np.dot(TO[:,0].T, UA[:,2], out=None) 

for i in range(3): 
    vec[i] = z_onto_y.item(0)*TO.item(i,1) + z_onto_x.item(0)*TO.item(i,0)    

z_onto_xy = np.matrix([[vec[0], vec[1], vec[2]]])


x_TO=np.array([0,0,0])
x_TO=TO[:,0]

z_onto_xy=np.matrix([UA[0,2],UA[1,2],0])


if arm==1: #right arm
    if relative_angle(z_onto_xy,TO[:,1].T)<math.pi/2:
        sign=-1
    else:
        sign=1
else:      #left arm
    if relative_angle(z_onto_xy,TO[:,1].T)<math.pi/2:
        sign=1
    else:
        sign=-1
POE = sign*relative_angle(arm*z_onto_xy, -x_TO.T) #right arm
        
# Angle of elevation 
AOE = relative_angle(UA[:,2].T,TO[:,2].T) #relative angle btw UA_y  and TO_y

# Humeral rotation 
rotPOE = matrix_op.rotZ(POE)#rotation around Z of POE 
rotAOE = matrix_op.rotY(-arm*AOE) #rotation around  of the AOE   
rotHR = np.matmul(np.matmul(np.matmul(rotAOE.T,rotPOE.T),TO.T),UA) #shoulder as ZXZ mechanism
HR = math.atan2(rotHR[1,0],(rotHR[1,1])) #arctg (sin/cos) given that HR is a rotation around z-axis

# Flexion extension 
FE = relative_angle(FA[:,2].T,UA[:,2].T) #relative angle between z axis

# Pronosupination 
rotFE=matrix_op.rotX(FE)
rotPS = np.matmul(np.matmul(rotFE.T,UA.T),FA) 

PS = math.atan2(rotPS[1,0], rotPS[1,1]) #pronosupination is a rotation around z axis 
        
if (AOE*180/3.14>155)|(AOE*180/3.14<25):
    warning=1
else:
    warning=0

#sign adjustment according to ISB standards
POE=arm*POE
HR=arm*HR
PS=-arm*PS





