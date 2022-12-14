#!/usr/bin/python3

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


# Select:
#  arm=1 for right arm
# arm=-1 for left arm 
arm =1

calibration_flag=-1

# BeagleBone Public IP address
for name, interface in ifcfg.interfaces().items():
    if interface['device'] == "wlan0":      # Device name
        IPAddr = interface['inet']          # First IPv4 found
        print("You are connected to the network. IP Address: ", IPAddr)         

# These are the IP addresses of each IMU. They are used to send commands to the IMUs
# You can find and modify them with the GUI. Please make sure they are correct 
# (they change from time to time)---> now they should be constant***
send_addresses = ["192.168.0.100","192.168.0.101","192.168.0.102"] 

# The send port is the same for each IMU and can be found in the GUI as the receive port of the IMU 
# The send port is the one that the IMUs listen to 
send_port = 9000

# Array of UDP ports to listen to, one per NGIMU.  These ports must be equal to
# the UDP Send Port in the NGIMU settings. This setting is changed
# automatically when connecting to the NGIMU using the NGIMU GUI. Please make sure they are correct 
# (they change from time to time)
receive_ports = [8100, 8101, 8102]
    
# Send /identify message to strobe all LEDs.  The OSC message is constructed
# from raw bytes as per the OSC specification.  The IP address must be equal to
# the IP address of the target NGIMU. The port must be equal to the Receive Port
# in the NGIMU UDP settings
print("Opening UDP socket...")


send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
IPAddr = "192.168.0.103"


for send_address in send_addresses:
    # Change IMUs UDP send addresses to match BeagleBone IP address
    IMU_client = udp_client.SimpleUDPClient(send_address, send_port)
    # Make the led blink
    IMU_client.send_message("/identify", 0.0)
    IMU_client.send_message("/wifi/send/ip", IPAddr) #IP address of the beaglebone (changed with IP address of the computer)
    if send_address == send_addresses[0]:
        print("Put this IMU on the trunk")
    elif send_address == send_addresses[1]:
        print("Put this IMU on the upper arm")
    elif send_address == send_addresses[2]:
        print("Put this IMU on the forearm")
    else:
        print("Error: the send address is not correct")
    time.sleep(1)

# Open the UDP connection to continuously read messages from the IMUs network
receive_sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(len(receive_ports))]

index = 0
for receive_socket in receive_sockets:
    receive_socket.bind(("", receive_ports[index]))
    index = index + 1
    receive_socket.setblocking(False)
    
print("Starting communication...")
time.sleep(0.5)

timecount=0

# Initialize matrix where to save IMUs data
TO_g=np.identity(3, dtype=float)
UA_g=np.identity(3, dtype=float)
FA_g=np.identity(3, dtype=float)
y_onto_xz = np.matrix([[0, 0, 0]])
vec = [0, 0, 0]

send_port = 9000
PC_client = udp_client.SimpleUDPClient(send_address, send_port)



#creation of the .csv file 

# current_datetime=datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
# str_current_datetime=str(current_datetime)
# name_isb= "ISB_Tiago_data_"+str_current_datetime+".csv"
# name_acc="ACCdata_"+str_current_datetime+".csv"
# name_rot="ROTdata_"+str_current_datetime+".csv"
# outdir = './DATA'
# if not os.path.exists(outdir):
#     os.mkdir(outdir)
# name_isbfile = os.path.join(outdir, name_isb) 
# name_accfile=os.path.join(outdir, name_acc) 
# name_rotfile=os.path.join(outdir, name_rot) 
# isb=open(name_isbfile,'w',encoding='UTF8')
# acc=open(name_accfile,'w',encoding='UTF8')
# rot_csv=open(name_rotfile,'w',encoding='UTF8')
# writer_isb=csv.writer(isb, delimiter=',')
# writer_acc=csv.writer(acc, delimiter=',')
# writer_rot=csv.writer(rot_csv, delimiter=',')
# header_isb=['time','POE','UAE','HR','FE','PS','J1','J2','J3','J4','J5']
# header_acc=['time','a_TOx','a_TOy','a_TOz','a_UAx','a_UAy','a_UAz','a_FAx','a_FAy','a_FAz']
# header_rot=['time','TOxx', 'TOyx','TOzx','TOxy' ,'TOyy', 'TOzy','TOxz' ,'TOyz' ,'TOzz','UAxx', 'UAyx','UAzx','UAxy' ,'UAyy', 'UAzy','UAxz' ,'UAyz' ,'UAzz','FAxx', 'FAyx','FAzx','FAxy' ,'FAyy', 'FAzy','FAxz' ,'FAyz' ,'FAzz']
# writer_isb.writerow(header_isb)
# writer_acc.writerow(header_acc)
# writer_rot.writerow(header_rot)

empty_mat=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
TO_npose=empty_mat
UA_npose=empty_mat
FA_npose=empty_mat
TO_tpose=empty_mat
UA_tpose=empty_mat
FA_tpose=empty_mat
n=0
i=0
m=0
j1_angle=0
j2_angle=0
j3_angle=0
j4_angle=0
j5_angle=0


while True:
    for udp_socket in receive_sockets: 
        try:
            data, addr = udp_socket.recvfrom(2048)
            
        except socket.error:
            pass
        else:
            for message in osc_decoder.decode(data):
                #print(message)                
                time_stamp = message[0]
                data_type = message[1]              
                if data_type == '/matrix': #this can be changed with every message avaible from the IMUs (gyro data, temperature...)
                    Rxx = message[2]
                    Ryx = message[3]
                    Rzx = message[4]
                    Rxy = message[5]
                    Ryy = message[6]
                    Rzy = message[7]
                    Rxz = message[8]
                    Ryz = message[9]
                    Rzz = message[10] 
                   
                    if udp_socket.getsockname()[1] == receive_ports[0]:
                        TO_g=np.matrix([[Rxx,Ryx,Rzx],[Rxy ,Ryy, Rzy],[Rxz ,Ryz ,Rzz]])               
                    elif udp_socket.getsockname()[1] == receive_ports[1]:       
                        UA_g=np.matrix([[Rxx,Ryx,Rzx],[Rxy ,Ryy, Rzy],[Rxz ,Ryz ,Rzz]])    
                    elif udp_socket.getsockname()[1] == receive_ports[2]:
                        FA_g=np.matrix([[Rxx,Ryx,Rzx],[Rxy ,Ryy, Rzy],[Rxz ,Ryz ,Rzz]])                     
                    else:
                        pass 

                if data_type =='/linear': #linear accelerations in IMU axis
                    a_x=message[2]
                    a_y=message[3]
                    a_z=message[4] 
                    if udp_socket.getsockname()[1] == receive_ports[0]:
                        a_TO=np.array([a_x,a_y,a_z])
                    elif udp_socket.getsockname()[1] == receive_ports[1]:       
                        a_UA=np.array([a_x,a_y,a_z])
                    elif udp_socket.getsockname()[1] == receive_ports[2]:
                        a_FA=np.array([a_x,a_y,a_z])
                    else:
                        pass 


            if calibration_flag==-1: #Settings for calibration 
                print("Press 'Enter' to start calibration (N-POSE+ T-POSE)\n")      
                while(keyboard.read_key() !="enter"): #wait for the patient to press Enter 
                    pass
                print("Stand still with arms along sides (N-pose)...\n") 
                calibration_flag=0
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

                time.sleep(2) 

            elif calibration_flag==0: #acquisition of calibration data
                if time.time()-start<10: #N-POSE data acquisiton 
                    sumTO_npose=sumTO_npose+TO_g  #to store all the matrix in the n-pose
                    sumUA_npose=sumUA_npose+UA_g          
                    sumFA_npose=sumFA_npose+FA_g
                    n=n+1

                elif (time.time()-start>10) & (time.time()-start<15): #Wait some time to change position
                    if i<1:
                        print("Raise arms to the side and keep them horizontal (T-pose)...\n")
                    i=i+1
                    
                elif (time.time()-start>15) & (time.time()-start<20): #T-POSE data acquisition 

                    sumUA_tpose=sumUA_tpose+UA_g
                    sumFA_tpose=sumFA_tpose+FA_g
                    m=m+1



                elif time.time()-start>20:
                    print("Calibration done!\n")
                    print("To calibrate again press 'Esc', otherwise press 'Enter'\n")
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


                    if keyboard.read_key() =="enter":
                        calibration_flag=1

                    elif keyboard.read_key()=="esc": 
                        calibration_flag=-1
                    
            elif calibration_flag==1:
                
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
    



            #Method 2 to evaluate the projection: 

        #   theta=relative_angle(np.squeeze(TO[:,0]),np.squeeze(UA[:,1]))  #relative angle between x torso and y ua 
        #   alpha=relative_angle(np.squeeze(TO[:,2]),np.squeeze(UA[:,1]))

        #   for i in range(3):
        #       vec[i] = math.cos(theta)*TO.item(i,0) + math.cos(alpha)*TO.item(i,2) 