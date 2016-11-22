# angles defined in the same way as in the coursework,
# but all angles are clockwise in the positive direction

# phi1, phi2, phi3 are q1, q2 and q3

import numpy as np
from config import lengths
from config import servo_angle_limits as lims


def get_sympy_jacobian():
	from sympy import Symbol, diff, sin, cos
	# Calculating J and pinvJ using symbolic algebra - only have to do this if J & pinvJ need to be recalculated
	qq1 = Symbol('qq1')
	qq2 = Symbol('qq2')
	qq3 = Symbol('qq3')
	qq4 = Symbol('qq4')


	x = -(lengths[0]*sin(qq1) + lengths[1]*sin(qq1+qq2) + lengths[2]*sin(qq1+qq2+qq3)+ lengths[3]*sin(qq1+qq2+qq3+qq4))
	y = lengths[0]*cos(qq1) + lengths[1]*cos(qq1+qq2) + lengths[2]*cos(qq1+qq2+qq3) + lengths[3]*cos(qq1+qq2+qq3+qq4)

	J1 =[diff(x,qq2), diff(x,qq3), diff(x,qq4)]
	J2 =[diff(y,qq2), diff(y,qq3), diff(y,qq4)]

	return np.array([J1, J2])

def pJ(qq):
	from numpy import sin, cos
	qq1 = 0
	qq2 = qq[0]
	qq3 = qq[1]
	qq4 = qq[2]
	#J1 = [-lengths[0]*cos(qq1) - lengths[1]*cos(qq1 + qq2) - lengths[2]*cos(qq1 + qq2 + qq3) - lengths[3]*cos(qq1 + qq2 + qq3 + qq4), -lengths[1]*cos(qq1 + qq2) - lengths[2]*cos(qq1 + qq2 + qq3) - lengths[3]*cos(qq1 + qq2 + qq3 + qq4), -lengths[2]*cos(qq1 + qq2 + qq3) - lengths[3]*cos(qq1 + qq2 + qq3 + qq4), -lengths[3]*cos(qq1 + qq2 + qq3 + qq4)]
	#J2 = [-lengths[0]*sin(qq1) - lengths[1]*sin(qq1 + qq2) - lengths[2]*sin(qq1 + qq2 + qq3) - lengths[3]*sin(qq1 + qq2 + qq3 + qq4), -lengths[1]*sin(qq1 + qq2) - lengths[2]*sin(qq1 + qq2 + qq3) - lengths[3]*sin(qq1 + qq2 + qq3 + qq4), -lengths[2]*sin(qq1 + qq2 + qq3) - lengths[3]*sin(qq1 + qq2 + qq3 + qq4), -lengths[3]*sin(qq1 + qq2 + qq3 + qq4)]
	J = np.array([
		[ -0.148*cos(qq1 + qq2) - 0.149*cos(qq1 + qq2 + qq3) - 0.139*cos(qq1 + qq2 + qq3 + qq4), -0.149*cos(qq1 + qq2 + qq3) - 0.139*cos(qq1 + qq2 + qq3 + qq4), -0.139*cos(qq1 + qq2 + qq3 + qq4)],
		[ -0.148*sin(qq1 + qq2) - 0.149*sin(qq1 + qq2 + qq3) - 0.139*sin(qq1 + qq2 + qq3 + qq4), -0.149*sin(qq1 + qq2 + qq3) - 0.139*sin(qq1 + qq2 + qq3 + qq4), -0.139*sin(qq1 + qq2 + qq3 + qq4)]
	])
	return np.linalg.pinv(J)

def f(qq):
	from numpy import sin, cos
	qq1 = 0
	qq2 = qq[0]
	qq3 = qq[1]
	qq4 = qq[2]
	#x = -(lengths[0]*sin(qq1) + lengths[1]*sin(qq1+qq2) + lengths[2]*sin(qq1+qq2+qq3)+ lengths[3]*sin(qq1+qq2+qq3+qq4))
	#y = lengths[0]*cos(qq1) + lengths[1]*cos(qq1+qq2) + lengths[2]*cos(qq1+qq2+qq3) + lengths[3]*cos(qq1+qq2+qq3+qq4)
	return np.array([-(lengths[0]*sin(qq1) + lengths[1]*sin(qq1+qq2) + lengths[2]*sin(qq1+qq2+qq3)+ lengths[3]*sin(qq1+qq2+qq3+qq4)), lengths[0]*cos(qq1) + lengths[1]*cos(qq1+qq2) + lengths[2]*cos(qq1+qq2+qq3) + lengths[3]*cos(qq1+qq2+qq3+qq4)], dtype=np.float64)

def get_servo_angles(r):
	# returns a set of servo values to send to the robot
	# the inputs are the coordinates for the desired location of the end effector

	# r is the desired coordinate of the foot that is not the base foot (the end effector)
	r = np.asarray(r)
	# change starting angles
	q1 = 0
	q2 = 2.0
	q3 = 2.0
	q4 = 2.0
	q = np.array([q2, q3, q4], dtype=np.float64)

	# find set of angles (q) to get to r
	for i in range(1,200):
		q = q + np.dot(pJ(q),(r-f(q)))

	# change q so that it is between 0 and 2pi
	q = q % (2*np.pi)

	#print(f(q)) # checking that end effector is in the right location for the new q
	return q[0:]


def get_servo_angles_for_list(r, q): #q is just q2, q3, q4
	r = np.asarray(r)

	for i in range (1,1000):
		q = q + np.dot(pJ(q),(r-f(q)))
	q = q % (2*np.pi)
	for i in range(0,3):
		if q[i]>np.pi:
			q[i] = q[i]-2*(np.pi)
	return q[0:]


listr = np.array([[0.2, 0.2], [0.3, 0.3], [0.35, 0.35], [0.4, 0.35]])
qstart = np.array([0.0, 0.0, 0.0])

def list_of_angles(listr, qstart): # list r needs to be a (2 x length) np array. 
	# qstart needs to be a np array with phi2, phi3 and phi4 in it (not including phi1 as it is assumed to be 0)
	l = listr.shape[0] # number of locations for the loop

	qreturn = np.zeros((l, 3))

	if listr.shape[1]!= 2:
		print('listr is in the wrong format - needs to be a 2 by (length) np array')
	if qstart.shape != (3,):
		print('qstart is in the wrong format - needs to be a np array with 3 elements in it')
	# start the algorithm off with qstart as the starting angles
	qreturn[0,:] = get_servo_angles_for_list(listr[0], qstart)

	for i in range(1,l): 	# make sure that it can get there
		qreturn[i,:] = get_servo_angles_for_list(listr[i], qreturn[(i-1),:]) # start off each run at the robot is currently
		if qreturn[i,0]<lims[0,0] or qreturn[i,0]>lims[0,1]:
			print('at index %d (python indexing starting at 0), q2 (%r) is out of range' % (i,qreturn[i,0]))
		if qreturn[i,1]<lims[1,0] or qreturn[i,1]>lims[1,1]:
			print('at index %d (python indexing starting at 0), q3 (%r) is out of range'% (i,qreturn[i,1]))
		if qreturn[i,2]<lims[2,0] or qreturn[i,2]>lims[2,1]:
			print('at index %d (python indexing starting at 0), q4 (%r) is out of range'% (i,qreturn[i,2]))
	return qreturn

def heart():
	heartr = np.zeros((104, 2))
	i = 0
	for t in np.arange(0.1, 2*np.pi, 0.06):
		x = 16*np.power(np.sin(t),3)
		y = 13*np.cos(t) - 5*np.cos(2*t) - 2*np.cos(3*t) - np.cos(4*t)
		heartr[i] = np.array([-0.329+0.0048*x, 0.24+0.007*y])
		i = i+1
	return heartr

qheart = list_of_angles(heart(),qstart)


if __name__ == '__main__':
	print(get_servo_angles([0.3,0.3]))
	print("Jacobian is", get_sympy_jacobian())
