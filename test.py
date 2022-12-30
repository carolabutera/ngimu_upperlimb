import matrix_op
import numpy as np 
import csv
import math
from numpy.linalg import norm

R_ga=matrix_op.rotZ(math.pi/6)
R_ab=matrix_op.rotY(math.pi/7)
R_gb=np.matmul(R_ga,R_ab)
a=[]
a=1
a=a,3

R_ab_2=np.matmul(R_ga.T,R_gb)

R_ga_2=np.matmul(R_gb,R_ab.T)
print("R_ga",R_ga_2)

print("R_ga",R_ga)
print(a)

max=max(a)