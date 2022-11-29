#!/usr/bin/python3

import time
import matrix_op
import numpy as np
import math
from numpy.linalg import norm
from datetime import datetime
import csv
   

# Function that computes the relative angle between two vectors:
def relative_angle(v1,v2):
    angle_rel = math.atan2(norm(np.cross(v1,v2),1),(np.dot(v1,np.transpose(v2))))
    return angle_rel

# Select:
#  arm=1 for right arm
# arm=-1 for left arm 
arm =-1


y_onto_xz = np.matrix([[0, 0, 0]])
vec = [0, 0, 0]

#Initial rotations: rotation matrices that depend on the position of the IMUs on the exosuit 
initRotTO=np.identity(3, dtype=float)
initRotUA=np.identity(3, dtype=float)
#initRotUA=matrix_op.rotY(-arm*math.pi/2)  #untoggle here 
initRotFA=np.identity(3, dtype=float)
#initRotFA=matrix_op.rotY(-arm*math.pi/2)  #untoggle here

#initialization of Tiago joint
j1_angle=0
j2_angle=0
j3_angle=0
j4_angle=0
j5_angle=0


 


#per linee da a .... 
with open("synchro_tiago_imu.csv", 'r') as file:
  csvreader = csv.reader(file)
  for row in csvreader:
    print(row)

sumTO=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
sumUA=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
sumFA=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
n=0
i=0
                   

TO_g=np.matmul(TO_g,initRotTO.T)                 
UA_g=np.matmul(UA_g,initRotUA.T)  
FA_g=np.matmul(FA_g, initRotFA.T)

sumTO=sumTO+TO_g
sumUA=sumUA+UA_g          
sumFA=sumFA+FA_g
n=n+1

 #T-POSE data acquisition 
sumTO=sumTO+TO_g        
sumUA=sumUA+UA_g*matrix_op.rotZ(arm*math.pi/2)
sumFA=sumFA+FA_g*matrix_op.rotZ(arm*math.pi/2)
n=n+1

 
meanTO=sumTO/n
meanUA=sumUA/n
meanFA=sumFA/n

#meanTO_isb=np.matmul(matrix_op.rotX(-math.pi/2),meanTO) #mean of the calibration matrix expressed in frame with y up
# meanUA_isb=np.matmul(matrix_op.rotX(-math.pi/2),meanUA)
# meanFA_isb=np.matmul(matrix_op.rotX(-math.pi/2),meanFA)
meanTO_isb=np.matmul(meanTO,matrix_op.rotX(math.pi/2)) #mean of the calibration matrix expressed in frame with y up
meanUA_isb=np.matmul(meanUA,matrix_op.rotX(math.pi/2))
meanFA_isb=np.matmul(meanFA,matrix_op.rotX(math.pi/2))


TO_isb=np.matmul(TO_g,matrix_op.rotX(math.pi/2)) #matrix read from IMU expressed in RF with y up
UA_isb=np.matmul(UA_g,matrix_op.rotX(math.pi/2))
FA_isb=np.matmul(FA_g, matrix_op.rotX(math.pi/2))

TO=np.matmul(meanTO_isb.T,TO_isb)
UA=np.matmul(meanUA_isb.T,UA_isb)
FA=np.matmul(meanFA_isb.T,FA_isb)

# POE
#Method 1 to evaluate projection:
y_onto_x=np.dot(TO[:,0].T, UA[:,1], out=None) 
y_onto_z=np.dot(TO[:,2].T, UA[:,1], out=None) 

for i in range(3): 
    vec[i] = y_onto_x.item(0)*TO.item(i,0) + y_onto_z.item(0)*TO.item(i,2)    

y_onto_xz = np.matrix([[vec[0], vec[1], vec[2]]])
x_TO=np.array([0,0,0])
x_TO=TO[:,0]

if arm==1: #right arm
    if relative_angle(y_onto_xz,TO[:,2].T)<math.pi/2:
        sign=-1
    else:
        sign=1
else:      #left arm
    if relative_angle(y_onto_xz,TO[:,2].T)<math.pi/2:
        sign=1
    else:
        sign=-1
POE = sign*relative_angle(arm*y_onto_xz, x_TO.T) #right arm
        
# Angle of elevation 
AOE = relative_angle(UA[:,1].T,TO[:,1].T) #relative angle btw UA_y  and TO_y

# Humeral rotation 
rotPOE = matrix_op.rotY(POE)#rotation around Y of POE 
rotAOE = matrix_op.rotZ(-arm*AOE) #rotation around Z of the AOE   
rotHR = np.matmul(np.matmul(np.matmul(rotAOE.T,rotPOE.T),TO.T),UA) #shoulder as YZY mechanism
HR = math.atan2(rotHR[0,2],(rotHR[0,0]))

# Flexion extension 
FE = relative_angle(FA[:,1].T,UA[:,1].T) #relative angle between y axis

# Pronosupination 
rotFE=matrix_op.rotX(FE)
rotPS = np.matmul(np.matmul(rotFE.T,UA.T),FA)

PS = math.atan2(rotPS[0,2], rotPS[0,0])
        
if (AOE*180/3.14>155)|(AOE*180/3.14<25):
    warning=1
else:
    warning=0

#sign adjustment according to ISB standards
POE=arm*POE
HR=arm*HR
PS=-arm*PS


print("POE: ", POE*180.0/3.14)                 
print("AOE: ", AOE*180.0/3.14)
print("HR: ",HR*180.0/3.14)
# print("FE: ",FE*180.0/3.14)                  
# print("PS: ",PS*180.0/3.14)
# print("a_TO", a_TO)
# print("a_UA", a_UA)

print(TO)
print(TO_g)

