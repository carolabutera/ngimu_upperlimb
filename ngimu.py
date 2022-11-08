import ngimu_utils as imu
#import numpy as np
import time

send_port = 9000
receive_port = 8001
ip_address = "192.168.0.101"

print("Opening UDP socket...")
receive_socket = imu.init_receive_socket(receive_port)
send_socket = imu.init_send_socket()

print("Flashing IMUs LED to indentify it")
message = imu.NGIMUCommands.identify
imu.send_message(send_socket, ip_address, send_port, message)

print("Reading IMU data...")
while True:
    options = "/matrix"
    message = imu.read_message(receive_socket, options)
    if message is None:
        print("none")
        pass
    else:
        #print("Time: ", time_stamp)
        #print(message)
        time_stamp = message[0]
        rot_matrix = message[2:-1]
        print("Elevation angle: ", rot_matrix[7])
