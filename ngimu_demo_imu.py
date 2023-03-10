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

# Select:
arm=1 # right arm
#arm=-1 # left arm 

ignore_magnetometer=1
# put =1 in case of acquisitions in noisy environment--> NB: align IMUs to each other before running the script!

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
IPAddr = "192.168.0.105"


for send_address in send_addresses:
    # Change IMUs UDP send addresses to match BeagleBone IP address
    IMU_client = udp_client.SimpleUDPClient(send_address, send_port)
    # Make the led blink
    IMU_client.send_message("/identify", 0.0)
    IMU_client.send_message("/wifi/send/ip", IPAddr) #IP address of the beaglebone (changed with IP address of the computer)
    if ignore_magnetometer==1:
        #IMU_client.send_message("/reset", True)
        IMU_client.send_message("/ahrs/magnetometer", True)

    if send_address == send_addresses[0]:
        print("Put this IMU on the trunk")
    elif send_address == send_addresses[1]:
        print("Put this IMU on the upper arm")
    elif send_address == send_addresses[2]:
        print("Put this IMU on the forearm")
    else:
        print("Error: the send address is not correct")
    time.sleep(10)

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
z_onto_xy = np.matrix([[0, 0, 0]])
vec = [0, 0, 0]
w_UA=[0,0,0]
v=[0,0,0]

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
# header_isb=['time','POE','UAE','HR','FE','PS']
# header_acc=['time','a_TOx','a_TOy','a_TOz','a_UAx','a_UAy','a_UAz','a_FAx','a_FAy','a_FAz']
# header_rot=['time','TOxx', 'TOyx','TOzx','TOxy' ,'TOyy', 'TOzy','TOxz' ,'TOyz' ,'TOzz','UAxx', 'UAyx','UAzx','UAxy' ,'UAyy', 'UAzy','UAxz' ,'UAyz' ,'UAzz','FAxx', 'FAyx','FAzx','FAxy' ,'FAyy', 'FAzy','FAxz' ,'FAyz' ,'FAzz']
# writer_isb.writerow(header_isb)
# writer_acc.writerow(header_acc)
# writer_rot.writerow(header_rot)


a_UA=[0,0,0]
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
                if data_type =='/sensors': #linear accelerations in IMU axis

                    w_x=message[2]
                    w_y=message[3]
                    w_z=message[4] 
                    if udp_socket.getsockname()[1] == receive_ports[0]:
                        w_TO=np.array([w_x,w_y,w_z])
                    elif udp_socket.getsockname()[1] == receive_ports[1]:       
                        w_UA=np.array([w_x,w_y,w_z])
                    elif udp_socket.getsockname()[1] == receive_ports[2]:
                        w_FA=np.array([w_x,w_y,w_z])
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
                n_to=0
                n_ua=0
                n_fa=0
                m_ua=0
                m_fa=0
                i=0

                time.sleep(2) 

            elif calibration_flag==0: #acquisition of calibration data
                if time.time()-start<10: #N-POSE data acquisiton 
                      #to store all the matrix in the n-pose
                           
                    
                    if udp_socket.getsockname()[1] == receive_ports[0]:
                        sumTO_npose=sumTO_npose+TO_g
                        n_to=n_to+1
                    elif udp_socket.getsockname()[1] == receive_ports[1]:       
                        sumUA_npose=sumUA_npose+UA_g
                        n_ua=n_ua+1
                    elif udp_socket.getsockname()[1] == receive_ports[2]:
                        sumFA_npose=sumFA_npose+FA_g
                        n_fa=n_fa+1

                elif (time.time()-start>10) & (time.time()-start<15): #Wait some time to change position
                    if i<1:
                        print("Raise arms to the side and keep them horizontal (T-pose)...\n")
                    i=i+1
                    
                elif (time.time()-start>15) & (time.time()-start<25): #T-POSE data acquisition 
                    if udp_socket.getsockname()[1] == receive_ports[1]:
                        sumUA_tpose=sumUA_tpose+UA_g
                        m_ua=m_ua+1
                    elif udp_socket.getsockname()[1] == receive_ports[2]:
                        sumFA_tpose=sumFA_tpose+FA_g
                        m_fa=m_fa+1



                elif time.time()-start>20:
                    print("Calibration done!\n")
                    print("To calibrate again press 'Esc', otherwise press 'Enter'\n")
                    TO_npose=sumTO_npose/n_to #mean of the matrix in the n-pose
                    UA_npose=sumUA_npose/n_ua
                    FA_npose=sumFA_npose/n_fa
                    #NB: n-poses are expressed wrt the global reference frame!

                    UA_tpose=sumUA_tpose/m_ua #mean of the matrix in the t-pose
                    FA_tpose=sumFA_tpose/m_fa
                    #NB: t-poses are expressed wrt the global reference frame!

                    #rotation matrix around the global axis to go from the n-pose to the t-pose (is the t-pose calibrated wrt to n-pose)
                    #theoretically it is np.matmul(TO_tpose, TO_npose.T) multiplied by an identity matrix which is the n_pose calibrated to itself
                    UA_tpose_calib=np.matmul(UA_tpose, UA_npose.T)
                    FA_tpose_calib=np.matmul(FA_tpose, FA_npose.T)

                    #alpha=angle between global y-axis and calibrated z-axis during t-pose
                    #alpha=relative_angle(-arm*UA_tpose_calib[:,2].T,[0,1,0]) #we can average values from FA and UA? 
                    UAz_onto_TOy=np.dot(TO_npose[:,1].T, UA_tpose_calib[:,2], out=None) 
                    UAz_onto_TOx=np.dot(TO_npose[:,0].T, UA_tpose_calib[:,2], out=None) 
                    
                    for i in range(3): 
                        v[i] = UAz_onto_TOy.item(0)*TO_npose.item(i,1) + UAz_onto_TOx.item(0)*TO_npose.item(i,0)    

                    UAz_onto_TOxy = np.matrix([[v[0], v[1], v[2]]])

                    alpha=relative_angle(-arm*UAz_onto_TOxy,[0,1,0])
                    #theta=angle between global x-axis and calibrated z-axis during t-pose
                    if alpha < math.pi/2:
                        #theta=relative_angle(-arm*UA_tpose_calib[:,2].T, [1,0,0])

                        theta=relative_angle(-arm*UAz_onto_TOxy, [1,0,0])

                    else:
                        #theta=2*math.pi-relative_angle(-arm*UA_tpose_calib[:,2].T, [1,0,0])
                        theta=2*math.pi-relative_angle(-arm*UAz_onto_TOxy, [1,0,0])

                    #matrix bewteen n-pose and body reference frame
                    TO_calib=np.matmul(matrix_op.rotZ(theta).T, TO_npose)
                    UA_calib=np.matmul(matrix_op.rotZ(theta).T, UA_npose)
                    FA_calib=np.matmul(matrix_op.rotZ(theta).T, FA_npose)
                    R_TO_ib=np.matmul(matrix_op.rotZ(theta).T, TO_npose)
                    R_UA_ib=np.matmul(matrix_op.rotZ(theta).T,UA_npose)
                    R_FA_ib=np.matmul(matrix_op.rotZ(theta).T,FA_npose)



                    if keyboard.read_key() =="enter":
                        calibration_flag=1

                    elif keyboard.read_key()=="esc": 
                        calibration_flag=-1
                    
            elif calibration_flag==1:
                
                TO_b=np.matmul(matrix_op.rotZ(theta).T,TO_g)#Tranform to place y-axis perpendicular to the torso                
                UA_b=np.matmul(matrix_op.rotZ(theta).T,UA_g)
                FA_b=np.matmul(matrix_op.rotZ(theta).T,FA_g)

                #Calibrated matrix (wrt to NPOSE initial position) expressed wrt to BODY reference frame
                TO=np.matmul(TO_b, TO_calib.T)  
                UA=np.matmul(UA_b, UA_calib.T)
                FA=np.matmul(FA_b, FA_calib.T)
                a_UA_c=np.matmul(R_UA_ib,a_UA)

                w_TO_c=np.matmul(R_TO_ib, w_TO)
                w_UA_c=np.matmul(R_UA_ib, w_UA)
                w_FA_c=np.matmul(R_FA_ib, w_FA)

                

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

            # z_onto_xy=np.matrix([UA[0,2],UA[1,2],0])

                
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




                PC_client.send_message("angle", AOE)

                #
                if timecount%500==0:
                    print("POE: ", POE*180.0/3.14)                 
                    print("AOE: ", AOE*180.0/3.14)                
                    print("HR: ",HR*180.0/3.14)
                    print("FE: ",FE*180.0/3.14)              
                    print("PS: ",PS*180.0/3.14)
                    # # print("a_UA_calib", a_UA_c)                
                    # print("a_UA", a_UA)
                    # print("a_UA_C", a_UA_c)
                    # print("w_UA", w_UA)
                    #print("w_UA_C", w_UA_c)
                    # if (AOE*180/3.14>155)|(AOE*180/3.14<25):
                    #     print("WARNING! POE and HR values are not accurate")

                # t=time.time()
                # isb_tiago_data=[t,POE*180.0/3.14,AOE*180.0/3.14,HR*180.0/3.14,FE*180.0/3.14,PS*180.0/3.14]
                # acc_data=[t,a_TO[0],a_TO[1],a_TO[2], a_UA[0],a_UA[1],a_UA[2],a_FA[0],a_FA[1],a_FA[2]]
                # rot_data=[t,TO_g[0,0],TO_g[0,1],TO_g[0,2],TO_g[1,0],TO_g[1,1],TO_g[1,2],TO_g[2,0],TO_g[2,1],TO_g[2,2],UA_g[0,0],UA_g[0,1],UA_g[0,2],UA_g[1,0],UA_g[1,1],UA_g[1,2],UA_g[2,0],UA_g[2,1],UA_g[2,2],FA_g[0,0],FA_g[0,1],FA_g[0,2],FA_g[1,0],FA_g[1,1],FA_g[1,2],FA_g[2,0],FA_g[2,1],FA_g[2,2]]
                # writer_isb.writerow(isb_tiago_data)
                # writer_acc.writerow(acc_data)
                # writer_rot.writerow(rot_data)

                timecount = timecount+1
        
