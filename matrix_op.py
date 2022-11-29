import numpy as np
import math
from numpy.linalg import norm

def rotX(alpha): 
    R=np.matrix([[1,0,0],
                 [0,math.cos(alpha),-math.sin(alpha)],
                 [0,math.sin(alpha),math.cos(alpha)]])
    return R
def rotY(alpha): 
    R=np.matrix([[math.cos(alpha),0,math.sin(alpha)],
                 [0,1,0],
                 [-math.sin(alpha),0,math.cos(alpha)]])
    return R

def rotZ(alpha): 
    R=np.matrix([[math.cos(alpha),-math.sin(alpha),0],
                 [math.sin(alpha),math.cos(alpha),0],
                 [0,0,1]])
    return R

def rotZ_T_rotX(alpha):
    R=np.matrix([[math.cos(alpha)*math.cos(alpha), math.sin(alpha), math.sin(alpha)*math.cos(alpha)],
                [-math.sin(alpha)*math.cos(alpha),math.cos(alpha),-math.sin(alpha)*math.sin(alpha)],
                [-math.sin(alpha),0,math.cos(alpha)]])

