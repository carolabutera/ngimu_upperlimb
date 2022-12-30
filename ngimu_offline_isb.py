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

def imu_to_isb(mat_imu):
    mat_isb=np.matmul(np.matmul(mat_imu, matrix_op.rotX(math.pi/2)), matrix_op.rotY(math.pi/2))
    return mat_isb

arm =1 #TIAGo arm

imu_tiago=open('IMUvsTIAGO_test2.csv','w',encoding='UTF8')

writer_imu_tiago=csv.writer(imu_tiago, delimiter=',',lineterminator='\n')

header_imu_tiago=['timestamp','POE_imu','POE_tiago','UAE_imu','UAE_tiago','HR_imu','HR_tiago','FE_imu','FE_tiago','PS_imu','PS_tiago','WARNING']

writer_imu_tiago.writerow(header_imu_tiago)


z_onto_xy = np.matrix([[0, 0, 0]])
vec = [0, 0, 0]
empty_mat=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
TO_npose=empty_mat
UA_npose=empty_mat
FA_npose=empty_mat
TO_tpose=empty_mat
UA_tpose=empty_mat
FA_tpose=empty_mat
sumTO_npose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
sumUA_npose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
sumFA_npose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
sumTO_tpose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
sumUA_tpose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
sumFA_tpose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
n=0
i=0
m=0


# N POSE
with open("./validation/test2/ROTdata_2022-12-13-12-25-46.csv", 'r') as file:
  csvreader = csv.reader(file)
  for index, row in enumerate(csvreader):

    if  index < 100 :
        if index>10: #first calibration pose (POE=0°,AOE=0°,HR=0°,FE=0°,PS=0°)
            TO_g=np.matrix([[float(row[1]),float(row[2]),float(row[3])],[float(row[4]),float(row[5]),float(row[6])],[float(row[7]),float(row[8]),float(row[9])]])
            UA_g=np.matrix([[float(row[10]),float(row[11]),float(row[12])],[float(row[13]),float(row[14]),float(row[15])],[float(row[16]),float(row[17]),float(row[18])]])
            FA_g=np.matrix([[float(row[19]),float(row[20]),float(row[21])],[float(row[22]),float(row[23]),float(row[24])],[float(row[25]),float(row[26]),float(row[27])]])
            sumTO_npose=sumTO_npose+TO_g  #to store all the matrix in the n-pose
            sumUA_npose=sumUA_npose+UA_g          
            sumFA_npose=sumFA_npose+FA_g
            n=n+1


# T POSE
with open("./validation/test2/synchro_tiago_imu_test2.csv", 'r') as file:
  csvreader = csv.reader(file)
  for index, row in enumerate(csvreader):
    if index==500:
        print("row", row)
    if  index < 773:
        if index> 446:  # second calibration pose (POE=0°,AOE=0°,HR=0°,FE=0°,PS=0°)
            UA_g=np.matrix([[float(row[10]),float(row[11]),float(row[12])],[float(row[13]),float(row[14]),float(row[15])],[float(row[16]),float(row[17]),float(row[18])]])
            FA_g=np.matrix([[float(row[19]),float(row[20]),float(row[21])],[float(row[22]),float(row[23]),float(row[24])],[float(row[25]),float(row[26]),float(row[27])]])
            sumUA_tpose=sumUA_tpose+UA_g
            sumFA_tpose=sumFA_tpose+FA_g
            m=m+1

 

TO_npose=sumTO_npose/n #mean of the matrix in the n-pose
UA_npose=sumUA_npose/n
FA_npose=sumFA_npose/n
print("to_npose",TO_npose)
print("ua_npose",UA_npose)
print("FA_npose",FA_npose)


UA_tpose=sumUA_tpose/m #mean of the matrix in the t-pose
FA_tpose=sumFA_tpose/m

print("UA_tpose",UA_tpose)
print("FA_tpose",FA_tpose)

#rotation matrix around the global axis to go from the n-pose to the t-pose (is the t-pose calibrated wrt to n-pose)
#theoretically it is np.matmul(TO_tpose, TO_npose.T) multiplied by an identity matrix which is the n_pose calibrated to itself
#UA_tpose_calib=np.matmul(UA_tpose, UA_npose.T)
UA_tpose_calib=np.matmul(FA_tpose, FA_npose.T)


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

with open("./validation/test2/synchro_tiago_imu_test2.csv", 'r') as file:

  csvreader = csv.reader(file)
  for index, row in enumerate(csvreader):
    if index>1:
        TO_g=np.matrix([[float(row[1]),float(row[2]),float(row[3])],[float(row[4]),float(row[5]),float(row[6])],[float(row[7]),float(row[8]),float(row[9])]])
        UA_g=np.matrix([[float(row[10]),float(row[11]),float(row[12])],[float(row[13]),float(row[14]),float(row[15])],[float(row[16]),float(row[17]),float(row[18])]])
        FA_g=np.matrix([[float(row[19]),float(row[20]),float(row[21])],[float(row[22]),float(row[23]),float(row[24])],[float(row[25]),float(row[26]),float(row[27])]])
        TO_b=np.matmul(matrix_op.rotZ(theta).T,TO_g)#Tranform to place y-axis perpendicular to the torso                
        UA_b=np.matmul(matrix_op.rotZ(theta).T,UA_g)
        FA_b=np.matmul(matrix_op.rotZ(theta).T,FA_g)


        #Calibrated matrix (wrt to NPOSE initial position) expressed wrt to z-upward reference frame
        TO_imu=np.matmul(TO_b, TO_calib.T)  
        UA_imu=np.matmul(UA_b, UA_calib.T)
        FA_imu=np.matmul(FA_b, FA_calib.T)

        TO=imu_to_isb(TO_imu)
        UA=imu_to_isb(UA_imu)
        FA=imu_to_isb(FA_imu)
    #NB: IMU reference frame:  y-axis perpendicular to torso, x-axis pointing to the right of the body and z-axis upward. 
    #    ISB reference frame has: y upward, x perpendicular to torso and y pointing to the right 


    # POE
    #projection of z-axis of upper arm onto xy plane of torso
        y_onto_x=np.dot(TO[:,0].T, UA[:,1], out=None) 
        y_onto_z=np.dot(TO[:,2].T, UA[:,1], out=None) 
        
        for i in range(3): 
            vec[i] = y_onto_x.item(0)*TO.item(i,0) + y_onto_z.item(0)*TO.item(i,2)    

        y_onto_xz = np.matrix([[vec[0], vec[1], vec[2]]])
        z_TO=np.array([0,0,0])
        z_TO=TO[:,2]
        
        if arm==1: #right arm
            if relative_angle(y_onto_xz,TO[:,0].T)<math.pi/2:
                sign=-1
            else:
                sign=1
        else:      #left arm
            if relative_angle(y_onto_xz,TO[:,0].T)<math.pi/2:
                sign=1
            else:
                sign=-1
        POE = sign*relative_angle(arm*y_onto_xz, -z_TO.T) 
                
        # Angle of elevation 
        AOE = relative_angle(UA[:,1].T,TO[:,1].T) #relative angle btw UA_y  and TO_y

        # Humeral rotation 
        rotPOE = matrix_op.rotY(POE)#rotation around Y of POE 
        rotAOE = matrix_op.rotX(-arm*AOE) #rotation around X of the AOE   
        rotHR = np.matmul(np.matmul(np.matmul(rotAOE.T,rotPOE.T),TO.T),UA) #shoulder as YZY mechanism
        HR = math.atan2(rotHR[0,2],(rotHR[0,0]))

        # Flexion extension 
        FE = relative_angle(FA[:,1].T,UA[:,1].T) #relative angle between y axis

        # Pronosupination 
        rotFE=matrix_op.rotZ(FE)
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

        imu_tiago_data=[row[0],POE, row[28],AOE, row[29],HR,row[30], FE,row[31],PS,row[32],warning]

        writer_imu_tiago.writerow(imu_tiago_data)

       