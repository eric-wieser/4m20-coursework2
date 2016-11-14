# x,y is location of end effector (i.e. the other foot)

# angles defined in the same way as in the coursework, 
# but all angles are clockwise in the positive direction

# phi1, phi2, phi3 are q1, q2 and q3

import numpy as np
from numpy import sin, cos
from numpy.linalg import pinv
from sympy import *
from sympy.matrices import *
from sympy.interactive.printing import init_printing
from functions import pJ, f

# the length of the links, in meters
lengths = np.array([
	0.125,
	0.148,
	0.149,
	0.139
])

qq1 = Symbol('qq1')
qq2 = Symbol('qq2')
qq3 = Symbol('qq3')
qq4 = Symbol('qq4')

x = -(lengths[0]*sin(qq1) + lengths[1]*sin(qq1+qq2) + lengths[2]*sin(qq1+qq2+qq3)+ lengths[3]*sin(qq1+qq2+qq3+qq4))
y = lengths[0]*cos(qq1) + lengths[1]*cos(qq1+qq2) + lengths[2]*cos(qq1+qq2+qq3) + lengths[3]*cos(qq1+qq2+qq3+qq4)

J1 =[diff(x,qq1), diff(x,qq2), diff(x,qq3), diff(x,qq4)]
J2 =[diff(y,qq1), diff(y,qq2), diff(y,qq3), diff(y,qq4)]

# r is the desired coordinate of the other foot
r = np.array([[0.4], [0.4]], dtype=np.float)

# change starting angles
q1 = 1.0
q2 = 1.0
q3 = 1.0
q4 = 1.0

q = np.array([q1, q2, q3, q4])

for i in range(1,1000):
	temp = np.array([[q[0]], [q[1]], [q[2]], [q[3]]])
	q = temp + np.dot(pJ(q),(r-f(q)))
	q = [q[0][0], q[1][0], q[2][0], q[3][0]]


print(f(q),q)



