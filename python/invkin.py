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

	lengths = [Symbol('lengths[{}]'.format(i)) for i in range(4)]

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


def get_servo_angles(r, q=np.zeros(3), tol=0.001):
	# returns a set of servo values to send to the robot
	# the inputs are the coordinates for the desired location of the end effector
	r = np.asarray(r)

	# gradient descent until convergence
	while True:
		diff = r - f(q)
		q = q + pJ(q) @ diff
		if np.linalg.norm(diff) < tol:
			break

	# normalize angles
	q = q % (2*np.pi)
	q[q > np.pi] -= 2*np.pi

	return q


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
	qreturn[0,:] = get_servo_angles(listr[0], qstart)

	for i in range(1,l): 	# make sure that it can get there
		qreturn[i,:] = get_servo_angles(listr[i], qreturn[(i-1),:]) # start off each run at the robot is currently
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

def remove_discontinuity(qlist):
	difference = qlist[0] - qlist[-1]
	num_extra_steps = int(np.ceil(max(difference/0.15)))
	step_difference = difference/num_extra_steps
	qpadding = np.zeros((num_extra_steps,3))
	qpadding[0] = qlist[-1] + step_difference
	for i in range(1, num_extra_steps):
		qpadding[i] = qpadding[i-1] + step_difference
		if qpadding[i,0]<lims[0,0] or qpadding[i,0]>lims[0,1]:
			print('at index %d (python indexing starting at 0), q2 (%r) is out of range' % (i,qpadding[i,0]))
		if qpadding[i,1]<lims[1,0] or qpadding[i,1]>lims[1,1]:
			print('at index %d (python indexing starting at 0), q3 (%r) is out of range'% (i,qpadding[i,1]))
		if qpadding[i,2]<lims[2,0] or qpadding[i,2]>lims[2,1]:
			print('at index %d (python indexing starting at 0), q4 (%r) is out of range'% (i,qpadding[i,2]))
	return qpadding

qheart = list_of_angles(heart(),qstart) # list of angles to draw a heart with discontinuity
qpadding = remove_discontinuity(qheart) # list of angles to get from qheart[-1] back to qheart[0]
qheart = np.concatenate((qheart,qpadding), axis=0) # list of angles to draw a heart with discontinuity removed


if __name__ == '__main__':
	q = get_servo_angles([0.3,0.3])
	np.testing.assert_allclose(f(q), [0.3, 0.3], rtol=0.001)

	print("Jacobian is", get_sympy_jacobian())

	try:
		from matplotlib import pyplot as plt
	except ImportError:
		import warnings
		warnings.warn("Can't plot the heart :(")
	else:
		r = heart()
		fig, ax = plt.subplots()
		ax.plot(r[:,0], r[:,1], label='Intended')
		r2 = np.array([f(q) for q in qheart])
		ax.plot(r2[:,0], r2[:,1], label='Calculated')
		ax.legend()
		ax.axis('equal')
		plt.show()
