#!/usr/bin/python3
#from multiprocessing.connection import answer_challenge
import osc_decoder
import socket
import time
import numpy as np
import math
from numpy import array
from numpy.linalg import norm

#function to calcute the relative angle:
def relative_angle(v1,v2):
    #check for sign
    angle_rel = math.atan2(norm(np.cross(v1,v2),1),(np.dot(v1,np.transpose(v2),)))
    return angle_rel
    

# Send /identify message to strobe all LEDs.  The OSC message is constructed
# from raw bytes as per the OSC specification.  The IP address must be equal to
# the IP address of the target NGIMU. The port must be equal to the Receive Port
# in the NGIMU UDP settings
print("Opening UDP socket...")
send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
send_addresses = ["192.168.0.100","192.168.0.101","192.168.0.102"]

index = 0
arm = 0 #1 for right arm, 0 or left 
for send_address in send_addresses:
    send_socket.sendto(bytes("/identify\0\0\0,\0\0\0", "utf-8"), (send_address, 9000)) #to make the led blink
    if send_address == send_addresses[0]:
        print("Put this IMU on the trunk")
    elif send_address == send_addresses[1]:
        print("Put this IMU on the upper arm")
    elif send_address == send_addresses[2]:
        print("Put this IMU on the forearm")
    else:
        print("Error: the send address does not correspond")
    time.sleep(2) #need to be set to 10 

# Array of UDP ports to listen to, one per NGIMU.  These ports must be equal to
# the UDP Send Port in the NGIMU settings.  The UDP Send IP Address setting
# must be the computer's IP address.  Both these settings are changed
# automatically when connecting to the NGIMU using the NGIMU GUI.

receive_ports = [8105, 8100, 8104]

receive_sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(len(receive_ports))]

index = 0
for receive_socket in receive_sockets:
    receive_socket.bind(("", receive_ports[index]))
    index = index + 1
    receive_socket.setblocking(False)
    

print("Starting communication...")
time.sleep(3)

timecount=0

TO=np.identity(3, dtype=float)
UA=np.identity(3, dtype=float)
FA=np.identity(3, dtype=float)
y_onto_xz = np.matrix([[0, 0, 0]])
a = [0, 0, 0]
#y_onto_xz = y_onto_xz.T

while True:
    imu_read = [0, 0, 0]
    #print("id", imuID)
    for udp_socket in receive_sockets: 
        try:
            data, addr = udp_socket.recvfrom(2048)
        except socket.error:
            #print("socket error:",socket.error.strerror)
            pass
        else:
            for message in osc_decoder.decode(data):
                #print("message ok")                
                time_stamp = message[0]
                data_type = message[1]              
                if data_type == '/matrix':
                    Rxx = message[2]
                    Ryx = message[3]
                    Rzx = message[4]
                    Rxy = message[5]
                    Ryy = message[6]
                    Rzy = message[7]
                    Rxz = message[8]
                    Ryz = message[9]
                    Rzz = message[10] 
                    
                    if udp_socket.getsockname()[1] == 8105:
                        TO=np.matrix([[Rxx,Ryx,Rzx],[Rxy ,Ryy, Rzy],[Rxz ,Ryz ,Rzz]]) 
                        imu_read[0] = 1                        
                    elif udp_socket.getsockname()[1] == 8100:       
                        UA=np.matrix([[Rxx,Ryx,Rzx],[Rxy ,Ryy, Rzy],[Rxz ,Ryz ,Rzz]])  
                        imu_read[1] = 1      
                    elif udp_socket.getsockname()[1] == 8104:
                        FA=np.matrix([[Rxx,Ryx,Rzx],[Rxy ,Ryy, Rzy],[Rxz ,Ryz ,Rzz]])
                        imu_read[2] = 1 
                    else:
                        pass                    
    #print("IMU read: ", imu_read)               
    #if np.array(imu_read).sum()==3: #all imu have been read

    #angle extraction
    #POE
    y_onto_x=np.dot(TO[:,0].T, UA[:,1], out=None)
    y_onto_z=np.dot(TO[:,2].T, UA[:,1], out=None)
    
    for i in range(3):
        a[i] = y_onto_x.item(0)*TO.item(i,0) + y_onto_z.item(0)*TO.item(i,2)  
          
    y_onto_xz = np.matrix([[a[0], a[1], a[2]]])
        
    x_TO=np.array([0,0,0])
    x_TO=TO[:,0]

    if arm == 0:
        POE = relative_angle(-y_onto_xz, x_TO.T) #right arm
    else:
        POE = relative_angle(y_onto_xz, x_TO.T) #left arm
            
    #angle of elevation 
    AOE = relative_angle(UA[:,1].T,TO[:,1].T) #relative agle btw UA_y  and TO_y

    #humeral rotation
    rotPOE = np.matrix([[math.cos(POE),0,math.sin(POE)],
                    [0,1,0],
                    [-math.sin(POE),0,math.cos(POE)]])#rotation around y of the POE
    rotAOE = np.matrix([[1,0,0],
                    [0,math.cos(AOE),-math.sin(AOE)],
                    [0,math.sin(AOE),math.cos(AOE)]]) #rotation around x of AOE
    rotHR = np.matmul(np.matmul(np.matmul(rotAOE.T,rotPOE.T),TO.T),UA)
    HR = math.atan2(rotHR[0,2], rotHR[0,0])
    
    #flexion extension 
    FE = relative_angle(FA[:,1].T,UA[:,1].T)

    #pronosupination 
    rotFE = np.matrix([[math.cos(FE),0,math.sin(FE)],
                    [0,1,0],
                    [-math.sin(FE),0,math.cos(FE)]]) 
    rotPS = np.matmul(np.matmul(rotFE.T,UA.T),FA)

    PS = math.atan2(rotPS[0,2], rotPS[0,0]) #to compute
    
    timecount = timecount+1
    if timecount%1000 == 0:
        #print("y_onto_x: ", y_onto_x)
        #print("y_onto_z: ", y_onto_z)
        print("POE",POE)
        #print("AOE",AOE)
        #print("HR",HR)
        #print("FE",FE)
        #print("PS",PS)
time.sleep(0.005)

    




   


           
           

