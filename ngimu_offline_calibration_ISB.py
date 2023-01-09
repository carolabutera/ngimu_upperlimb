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

test=6

if test==1:
  synchro_file='./validation/test1/synchro_tiago_imu_test1.csv'
  tiago_data="./validation/test1/TIAGo_LogFile_2022_12_13_12_11_29.csv"
  rot_data="./validation/test1/ROTdata_2022-12-13-12-10-34.csv"
  file_name="./validation/test1/IMUvsTIAGO_ISB_test1.csv"
  low_n=1
  high_n=100

if test==2:
  synchro_file='./validation/test2/synchro_tiago_imu_test2.csv'
  tiago_data="./validation/test2/TIAGo_LogFile_2022_12_13_12_26_12.csv"
  rot_data="./validation/test2/ROTdata_2022-12-13-12-25-46.csv"
  file_name="./validation/test2/IMUvsTIAGO_ISB_test2.csv"
  low_n=1
  high_n=100

if test==3: 
  synchro_file='./validation/test3/synchro_tiago_imu_test3.csv'
  tiago_data="./validation/test3/TIAGo_LogFile_2022_12_13_12_37_58.csv"
  rot_data="./validation/test3/ROTdata_2022-12-13-12-37-37.csv"
  file_name="./validation/test3/IMUvsTIAGO_ISB_test3.csv"
  low_n=1
  high_n=100

if test==4:
  synchro_file='./validation/test4/synchro_tiago_imu_test4.csv'
  tiago_data="./validation/test4/TIAGo_LogFile_2022_12_20_17_05_05.csv"
  rot_data="./validation/test4/ROTdata_2022-12-20-17-01-06.csv"
  file_name="./validation/test4/IMUvsTIAGO_ISB_test4.csv"
  low_n=100
  high_n=200

if test==5:
  synchro_file='./validation/test5/synchro_tiago_imu_test5.csv'
  tiago_data="./validation/test5/TIAGo_LogFile_2022_12_20_17_29_11.csv"
  rot_data="./validation/test5/ROTdata_2022-12-20-17-25-40.csv"
  file_name="./validation/test5/IMUvsTIAGO_ISB_test5.csv"
  low_n=400
  high_n=500

if test==6:
  synchro_file='./validation/test6/synchro_tiago_imu_test6.csv'
  tiago_data="./validation/test6/TIAGo_LogFile_2022_12_20_17_41_37.csv"
  rot_data="./validation/test6/ROTdata_2022-12-20-17-41-10.csv"
  file_name="./validation/test6/IMUvsTIAGO_ISB_test6.csv"
  low_n=100
  high_n=200

if test==7:
  synchro_file='./validation/test7/synchro_tiago_imu_test7.csv'
  tiago_data="./validation/test7/TIAGo_LogFile_2022_12_20_18_04_56.csv"
  rot_data="./validation/test7/ROTdata_2022-12-20-18-03-41.csv"
  file_name="./validation/test7/IMUvsTIAGO_ISB_test7.csv"
  low_n=0
  high_n=100

arm =1 #TIAGo arm

imu_tiago=open(file_name,'w',encoding='UTF8')

writer_imu_tiago=csv.writer(imu_tiago, delimiter=',',lineterminator='\n')

header_imu_tiago=['timestamp','POE_imu','POE_tiago','UAE_imu','UAE_tiago','HR_imu','HR_tiago','FE_imu','FE_tiago','PS_imu','PS_tiago','WARNING']

writer_imu_tiago.writerow(header_imu_tiago)

calib_matrix=open("caliibrated_matrix.csv",'w',encoding='UTF8')
writer_calib_matrix=csv.writer(calib_matrix, delimiter=',',lineterminator='\n')
header_calib_matrix=['timestamp','TOxx', 'TOyx','TOzx','TOxy' ,'TOyy', 'TOzy','TOxz' ,'TOyz' ,'TOzz','UAxx', 'UAyx','UAzx','UAxy' ,'UAyy', 'UAzy','UAxz' ,'UAyz' ,'UAzz','FAxx', 'FAyx','FAzx','FAxy' ,'FAyy', 'FAzy','FAxz' ,'FAyz' ,'FAzz']

z_onto_xy = np.matrix([[0, 0, 0]])
vec = [0, 0, 0]
v = [0, 0, 0]
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
with open(rot_data, 'r') as file:
  csvreader = csv.reader(file)
  for index, row in enumerate(csvreader):

    if  index < high_n : #500 if test 5, 200 for others
        if index > low_n: #400 if test 5 ,100 for others
            TO_g=np.matrix([[float(row[1]),float(row[2]),float(row[3])],[float(row[4]),float(row[5]),float(row[6])],[float(row[7]),float(row[8]),float(row[9])]])
            UA_g=np.matrix([[float(row[10]),float(row[11]),float(row[12])],[float(row[13]),float(row[14]),float(row[15])],[float(row[16]),float(row[17]),float(row[18])]])
            FA_g=np.matrix([[float(row[19]),float(row[20]),float(row[21])],[float(row[22]),float(row[23]),float(row[24])],[float(row[25]),float(row[26]),float(row[27])]])
            sumTO_npose=sumTO_npose+TO_g  #to store all the matrix in the n-pose
            sumUA_npose=sumUA_npose+UA_g          
            sumFA_npose=sumFA_npose+FA_g
            n=n+1


# T POSE
with open(synchro_file, 'r') as file:
  csvreader = csv.reader(file)
  for index, row in enumerate(csvreader):# second calibration pose (POE=0°,AOE=0°,HR=0°,FE=0°,PS=0°)
    if index>0:
      if (float(row[28])<0.001)&(float(row[29])>1.56)&(float(row[29])<1.58)&(float(row[30])>1.56)&(float(row[30])<1.58)&(float(row[31])<0.001)&(float(row[32])<0.001):
        UA_g=np.matrix([[float(row[10]),float(row[11]),float(row[12])],[float(row[13]),float(row[14]),float(row[15])],[float(row[16]),float(row[17]),float(row[18])]])
        FA_g=np.matrix([[float(row[19]),float(row[20]),float(row[21])],[float(row[22]),float(row[23]),float(row[24])],[float(row[25]),float(row[26]),float(row[27])]])
        sumUA_tpose=sumUA_tpose+UA_g
        sumFA_tpose=sumFA_tpose+FA_g
        m=m+1

 

TO_npose=sumTO_npose/n #mean of the matrix in the n-pose
UA_npose=sumUA_npose/n
FA_npose=sumFA_npose/n


UA_tpose=sumUA_tpose/m #mean of the matrix in the t-pose
FA_tpose=sumFA_tpose/m

#rotation matrix around the global axis to go from the n-pose to the t-pose (is the t-pose calibrated wrt to n-pose)
#theoretically it is np.matmul(TO_tpose, TO_npose.T) multiplied by an identity matrix which is the n_pose calibrated to itself
UA_tpose_calib=np.matmul(UA_tpose, UA_npose.T)
FA_tpose_calib=np.matmul(FA_tpose, FA_npose.T)


#alpha=angle between global y-axis and calibrated z-axis during t-pose
alpha=relative_angle(-arm*UA_tpose_calib[:,2].T,[0,1,0]) #we can average values from FA and UA? 

UAz_onto_TOy=np.dot(TO_npose[:,1].T, UA_tpose_calib[:,2], out=None) 
UAz_onto_TOx=np.dot(TO_npose[:,0].T, UA_tpose_calib[:,2], out=None) 

for i in range(3): 
    v[i] = UAz_onto_TOy.item(0)*TO_npose.item(i,1) + UAz_onto_TOx.item(0)*TO_npose.item(i,0)    

UAz_onto_TOxy = np.matrix([[v[0], v[1], v[2]]])


#theta=angle between global x-axis and calibrated z-axis during t-pose
if alpha < math.pi/2:
    #theta=relative_angle(-arm*UA_tpose_calib[:,2].T, [1,0,0])
    theta=relative_angle(-arm*UAz_onto_TOxy, [1,0,0])

else:
    #theta=2*math.pi-relative_angle(-arm*UA_tpose_calib[:,2].T, [1,0,0])
    theta=2*math.pi-relative_angle(-arm*UAz_onto_TOxy.T, [1,0,0])



#matrix bewteen n-pose and body reference frame
TO_calib=np.matmul(matrix_op.rotZ(theta).T, TO_npose)
UA_calib=np.matmul(matrix_op.rotZ(theta).T, UA_npose)
FA_calib=np.matmul(matrix_op.rotZ(theta).T, FA_npose)

with open(synchro_file, 'r') as file:

  csvreader = csv.reader(file)
  for index, row in enumerate(csvreader):
    if index>0:
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
        calib_matrix=[row[0],TO[0,0],TO[0,1],TO[0,2],TO[1,0],TO[1,1],TO[1,2],TO[2,0],TO[2,1],TO[2,2],UA[0,0],UA[0,1],UA[0,2],UA[1,0],UA[1,1],UA[1,2],UA[2,0],UA[2,1],UA[2,2],FA[0,0],FA[0,1],FA[0,2],FA[1,0],FA[1,1],FA[1,2],FA[2,0],FA[2,1],FA[2,2]]
        writer_imu_tiago.writerow(imu_tiago_data)
        writer_calib_matrix.writerow(calib_matrix)

       