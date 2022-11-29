import numpy as np
import matrix_op
import math

a=matrix_op.rotX(math.pi/3)
c=matrix_op.rotZ(math.pi/2)
b=np.matmul(c,a)
d=np.matmul(a.T,b)
e=np.matmul(a,b.T)
f=np.matmul(b,a.T)
g=np.matmul(b.T,a)
h=np.matmul(a.T,b.T)

print('c',c)
print('d',d)
print('e',e)
print('f',f)
print('g',g)
print('h',h)