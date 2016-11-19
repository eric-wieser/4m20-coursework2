# angles defined in the same way as in the coursework,
# but all angles are clockwise in the positive direction

# phi1, phi2, phi3 are q1, q2 and q3

import numpy as np
from functions import pJ, f

# the length of the links, in meters
from config import lengths

def get_sympy_jacobian():
	from sympy import Symbol, diff, sin, cos
	# Calculating J and pinvJ using symbolic algebra - only have to do this if J & pinvJ need to be recalculated
	qq1 = Symbol('qq1')
	qq2 = Symbol('qq2')
	qq3 = Symbol('qq3')
	qq4 = Symbol('qq4')


	x = -(lengths[0]*sin(qq1) + lengths[1]*sin(qq1+qq2) + lengths[2]*sin(qq1+qq2+qq3)+ lengths[3]*sin(qq1+qq2+qq3+qq4))
	y = lengths[0]*cos(qq1) + lengths[1]*cos(qq1+qq2) + lengths[2]*cos(qq1+qq2+qq3) + lengths[3]*cos(qq1+qq2+qq3+qq4)

	J1 =[diff(x,qq1), diff(x,qq2), diff(x,qq3), diff(x,qq4)]
	J2 =[diff(y,qq1), diff(y,qq2), diff(y,qq3), diff(y,qq4)]

	return np.array([J1, J2])

def get_servo_angles(xcoord,ycoord):
	from numpy import sin, cos

	# returns a set of servo values to send to the robot
	# the inputs are the coordinates for the desired location of the end effector

	# r is the desired coordinate of the foot that is not the base foot (the end effector)
	r = np.array([xcoord, ycoord], dtype=np.float64)
	# change starting angles
	q1 = 2.0
	q2 = 2.0
	q3 = 2.0
	q4 = 2.0
	q = np.array([q1, q2, q3, q4], dtype=np.float64)

	# find set of angles (q) to get to r
	for i in range(1,200):
		q = q + np.dot(pJ(q),(r-f(q)))

	# change q so that it is between 0 and 2pi
	q = q % 2*np.pi

	# print(f(q)) # checking that end effector is in the right location for the new q

	# remember q[0][0] is phi1, q[1][0] is phi2, q[2][0] is phi3, q[3][0] is phi4
	return q[1:]

if __name__ == '__main__':
	print(get_servo_angles(0.3,0.3))
	print("Jacobian is", get_sympy_jacobian())
