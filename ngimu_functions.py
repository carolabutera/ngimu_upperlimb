
import osc_decoder
import socket
import time
import matrix_op
import numpy as np
import math
import csv
import matplotlib.pyplot as plt 
import matplotlib.animation as animation 
from numpy.linalg import norm
from datetime import datetime
from pythonosc import udp_client
import ifcfg
import json
import keyboard
import os

    

# Function that computes the relative angle between two vectors:
def relative_angle(v1,v2):
    angle_rel = math.atan2(norm(np.cross(v1,v2),1),(np.dot(v1,np.transpose(v2))))
    return angle_rel

def imu_to_isb(mat_imu):
    mat_isb=np.matmul(np.matmul(mat_imu, matrix_op.rotX(math.pi/2)), matrix_op.rotY(math.pi/2))
    return mat_isb


def start_npose()
    #start gopro recording (save as n_pose, subjet N)
    start=time.time()
    sumTO_npose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
    sumUA_npose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
    sumFA_npose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
    sumTO_tpose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
    sumUA_tpose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
    sumFA_tpose=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
    n=0
    m=0
    i=0

    if (time.time()-start<10): #N-POSE data acquisiton 
        sumTO_npose=sumTO_npose+TO_g  #to store all the matrix in the n-pose
        sumUA_npose=sumUA_npose+UA_g          
        sumFA_npose=sumFA_npose+FA_g
        n=n+1
    else 
    print("N-pose acquired")

def start_tpose()
#Start gopro recording
    start=time.time()
    if(time.time()-start<10):
        sumUA_tpose=sumUA_tpose+UA_g
        sumFA_tpose=sumFA_tpose+FA_g
        m=m+1

    else
        print("T_pose acquired")
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



def start_acquisition () #in the args mput in the name of the acquisition so that at the end of the acquisition i can choose to redo
 #press to stop                
# until i press button go on with the acquisition 

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

    t=time.time()

    PC_client.send_message("angle", AOE)

    if timecount%500==0:
        print("POE: ", POE*180.0/3.14)                 
        print("AOE: ", AOE*180.0/3.14)
        print("HR: ",HR*180.0/3.14)
        # # print("FE: ",FE*180.0/3.14)                  
        # # print("PS: ",PS*180.0/3.14)
        # # print("a_TO", a_TO)
        # # print("a_UA", a_UA)

        if (AOE*180/3.14>155)|(AOE*180/3.14<25):
            print("WARNING! POE and HR values are not accurate")

    # isb_tiago_data=[t,POE*180.0/3.14,AOE*180.0/3.14,HR*180.0/3.14,FE*180.0/3.14,PS*180.0/3.14,j1_angle*180.0/3.14,j2_angle*180.0/3.14,j3_angle*180.0/3.14,j4_angle*180.0/3.14,j5_angle*180.0/3.14]
    # acc_data=[t,a_TO[0],a_TO[1],a_TO[2], a_UA[0],a_UA[1],a_UA[2],a_FA[0],a_FA[1],a_FA[2]]
    # rot_data=[t,TO_g[0,0],TO_g[0,1],TO_g[0,2],TO_g[1,0],TO_g[1,1],TO_g[1,2],TO_g[2,0],TO_g[2,1],TO_g[2,2],UA_g[0,0],UA_g[0,1],UA_g[0,2],UA_g[1,0],UA_g[1,1],UA_g[1,2],UA_g[2,0],UA_g[2,1],UA_g[2,2],FA_g[0,0],FA_g[0,1],FA_g[0,2],FA_g[1,0],FA_g[1,1],FA_g[1,2],FA_g[2,0],FA_g[2,1],FA_g[2,2]]
    # writer_isb.writerow(isb_tiago_data)
    # writer_acc.writerow(acc_data)
    # writer_rot.writerow(rot_data)

    timecount = timecount+1

    #when button is pressed download the file from the gopro 
    
