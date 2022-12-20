#!/usr/bin/python3

#script to receive data from imus and save them into a csv file 

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


ignore_magnetometer=1 # put =1 in case of acquisitions in noisy environment--> NB: align IMUs to each other before running the script!


timecount=0
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
IPaddr = "192.168.0.103"

for send_address in send_addresses:
    # Change IMUs UDP send addresses to match BeagleBone IP address
    IMU_client = udp_client.SimpleUDPClient(send_address, send_port)
    # Make the led blink
    IMU_client.send_message("/identify", 0.0)
    IMU_client.send_message("/wifi/send/ip", IPaddr) #IP address of the beaglebone (changed with IP address of the computer)
    if ignore_magnetometer==1: 
        IMU_client.send_message("/reset", True)
        IMU_client.send_message("/ahrs/magnetometer", True)
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
time.sleep(0.5)

timecount=0

# Initialize matrix where to save IMUs data
TO_g=np.identity(3, dtype=float)
UA_g=np.identity(3, dtype=float)
FA_g=np.identity(3, dtype=float)

send_port = 9000
PC_client = udp_client.SimpleUDPClient(send_address, send_port)


#creation of the .csv file 
current_datetime=datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
str_current_datetime=str(current_datetime)

name_rotfile="ROTdata_"+str_current_datetime+".csv"

rot_csv=open(name_rotfile,'w',encoding='UTF8')

writer_rot=csv.writer(rot_csv, delimiter=',')

header_rot=['time','TOxx', 'TOyx','TOzx','TOxy' ,'TOyy', 'TOzy','TOxz' ,'TOyz' ,'TOzz','UAxx', 'UAyx','UAzx','UAxy' ,'UAyy', 'UAzy','UAxz' ,'UAyz' ,'UAzz','FAxx', 'FAyx','FAzx','FAxy' ,'FAyy', 'FAzy','FAxz' ,'FAyz' ,'FAzz']

writer_rot.writerow(header_rot)

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
             
            while(keyboard.read_key() !="enter"): #wait for the patient to press Enter 
                pass

            t=time.time()

            rot_data=[t,TO_g[0,0],TO_g[0,1],TO_g[0,2],TO_g[1,0],TO_g[1,1],TO_g[1,2],TO_g[2,0],TO_g[2,1],TO_g[2,2],UA_g[0,0],UA_g[0,1],UA_g[0,2],UA_g[1,0],UA_g[1,1],UA_g[1,2],UA_g[2,0],UA_g[2,1],UA_g[2,2],FA_g[0,0],FA_g[0,1],FA_g[0,2],FA_g[1,0],FA_g[1,1],FA_g[1,2],FA_g[2,0],FA_g[2,1],FA_g[2,2]]

            writer_rot.writerow(rot_data)

            if timecount%500==0:
                print("TO_G",TO_g)
                print("UA_G",UA_g)
                print("FA_g",FA_g)
            timecount=timecount+1
            
            




