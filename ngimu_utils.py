#!/usr/bin/python3

from distutils.log import error
from queue import Empty
import osc_decoder
import socket


class NGIMUData:
    def __init__(self, time_stamp, quaternions, rot_matrix, euler_angles, linear_acc):
        self.time_stamp = time_stamp
        self.quaternions = quaternions
        self.rot_matrix = rot_matrix
        self.euler_angles = euler_angles
        self.linear_acc = linear_acc

class NGIMUCommands:
        reset = "/reset\0\0\0,\0\0\0"
        sleep = "/sleep\0\0\0,\0\0\0"
        identify = "/identify\0\0\0,\0\0\0"
        apply = "/apply\0\0\0,\0\0\0"
        default = "/default\0\0\0,\0\0\0"
        ahrs_zero = "/ahrs_zero\0\0\0,\0\0\0"

def init_send_socket():
    send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return send_socket

def init_receive_socket(receive_port):
    receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    index = 0
    receive_socket.bind(("", receive_port))
    receive_socket.setblocking(False)
    return receive_socket

def send_message(send_socket, ip_address, send_port, message):
    send_socket.sendto(bytes(message, "utf-8"), (ip_address, send_port))

def read_message(receive_socket, options):
    try:
        data, addr = receive_socket.recvfrom(2048)
    except socket.error:
        pass
    else:
        for message in osc_decoder.decode(data):
            data_type = message[1]
            if data_type == options:
                #print(message)
                return message

def read_message2(receive_socket, options):
    data_packet = NGIMUData()
    attributes = []
    while(data_packet is Empty): #put here function that recognises if the data_packet is filled
        try:
            data, addr = receive_socket.recvfrom(2048)
        except socket.error:
            pass
        else:
            for message in osc_decoder.decode(data):
                data_type = message[1]
                if data_type == options:
                    #print(message)
                    return message

                data_packet.time_stamp = message[0]
                data_type = message[1]
                data = message[2:-1]
                if data_type == '/quaternion':
                    data_packet.quaternions = data
                    # data = [q1, q2, q3, q4]
                elif data_type == '/matrix':
                    data_packet.rot_matrix = data
                    # data = [Rxx, Rxy, Rxz, Ryx, Ryy, Ryz, Rzx, Rzy, Rzz]
                elif data_type == '/euler':
                    data_packet.euler_angles = data
                    # data = [roll, pitch, yaw]
                elif data_type == '/linear':
                    data_packet.linear_acc = data
                    # data = [ax, ay, az]
                elif data_type == '/error':
                    return error  
    return data_packet