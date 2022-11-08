#!/usr/bin/python3

import osc_decoder
import socket
import time
import matrix_op
import numpy as np
import math
from numpy.linalg import norm

# Definition of the function to compute the relative angle between two vectors:
def relative_angle(v1,v2):
    angle_rel = math.atan2(norm(np.cross(v1,v2),1),(np.dot(v1,np.transpose(v2),)))
    return angle_rel

# Select 0 for right arm, 1 for left arm 
arm = 1

# These are the IP addresses of each IMU. They are used to send commands to the IMUs
# You can find and modify them with the GUI. Please make sure they are correct 
# (they change from time to time)
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

for send_address in send_addresses:
    # Make the led blink
    send_socket.sendto(bytes("/identify\0\0\0,\0\0\0", "utf-8"), (send_address, send_port)) 
    if send_address == send_addresses[0]:
        print("Put this IMU on the trunk")
    elif send_address == send_addresses[1]:
        print("Put this IMU on the upper arm")
    elif send_address == send_addresses[2]:
        print("Put this IMU on the forearm")
    else:
        print("Error: the send address is not correct")
    time.sleep(5)

# Open the UDP connection to continuously read messages from the IMUs network
receive_sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(len(receive_ports))]

index = 0
for receive_socket in receive_sockets:
    receive_socket.bind(("", receive_ports[index]))
    index = index + 1
    receive_socket.setblocking(False)
    
print("Starting communication...")
time.sleep(1)

timecount=0

# Initialize matrix where to save IMUs data
TO=np.identity(3, dtype=float)
UA=np.identity(3, dtype=float)
FA=np.identity(3, dtype=float)
y_onto_xz = np.matrix([[0, 0, 0]])
vec = [0, 0, 0]

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
                if data_type == '/matrix': #this can be changed with every message avaible from the IMUs
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
                        TO=np.matrix([[Rxx,Ryx,Rzx],[Rxy ,Ryy, Rzy],[Rxz ,Ryz ,Rzz]])                        
                    elif udp_socket.getsockname()[1] == receive_ports[1]:       
                        UA=np.matrix([[Rxx,Ryx,Rzx],[Rxy ,Ryy, Rzy],[Rxz ,Ryz ,Rzz]])        
                    elif udp_socket.getsockname()[1] == receive_ports[2]:
                        FA=np.matrix([[Rxx,Ryx,Rzx],[Rxy ,Ryy, Rzy],[Rxz ,Ryz ,Rzz]])
                    else:
                        pass                    

    # Angles extraction
    # POE

#Method 1 to evaluate projection:
    y_onto_x=np.dot(TO[:,0].T, UA[:,1], out=None) 
    y_onto_z=np.dot(TO[:,2].T, UA[:,1], out=None) 
    
    for i in range(3): 
        vec[i] = y_onto_x.item(0)*TO.item(i,0) + y_onto_z.item(0)*TO.item(i,2)  

#Method 2 to evaluate the projection: 

 #   theta=relative_angle(np.squeeze(TO[:,0]),np.squeeze(UA[:,1]))  #relative angle between x torso and y ua 
 #   alpha=relative_angle(np.squeeze(TO[:,2]),np.squeeze(UA[:,1]))

 #   for i in range(3):
 #       vec[i] = math.cos(theta)*TO.item(i,0) + math.cos(alpha)*TO.item(i,2)   

    y_onto_xz = np.matrix([[vec[0], vec[1], vec[2]]])
    x_TO=np.array([0,0,0])
    x_TO=TO[:,0]

    if arm == 0:
        POE = relative_angle(y_onto_xz, x_TO.T) #right arm
    else:
        POE = relative_angle(-y_onto_xz, x_TO.T) #left arm
            
    # Angle of elevation 
    AOE = relative_angle(UA[:,1].T,TO[:,1].T) #relative agle btw UA_y  and TO_y

    # Humeral rotation 
    rotAOE = matrix_op.rotZ(AOE) #rotation around X of the AOE   
    rotPOE = matrix_op.rotY(POE)#rotation around Y of POE         
    rotHR = np.matmul(np.matmul(np.matmul(rotAOE.T,rotPOE.T),TO.T),UA) #shoulder as YZY mechanism

    HR = math.atan2(rotHR[0,2], rotHR[0,0])
 
    
    # Flexion extension 
    FE = relative_angle(FA[:,1].T,UA[:,1].T)

    # Pronosupination 
    rotFE = np.matrix([[1,0,0],
                    [0, math.cos(FE), -math.sin(FE)],
                    [0, math.sin(FE), math.cos(FE)]]) #rotation around x of FE
    rotPS = np.matmul(np.matmul(rotFE.T,UA.T),FA)

    PS = math.atan2(rotPS[0,2], rotPS[0,0])
    
    if timecount%1000 == 0:
        print("POE: ", POE*180.0/3.14)
        print("AOE: ", AOE*180.0/3.14)
        print("HR: ",HR*180.0/3.14)
        #print("FE: ",FE*180.0/3.14)
        #print("PS: ",PS*180.0/3.14)
        if (AOE*180/3.14>155)|(AOE*180/3.14<25):
            print("WARNING! POE and HR values are not accurate")


    timecount = timecount+1 



   


           
           

