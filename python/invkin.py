# angles defined in the same way as in the coursework,
# but all angles are clockwise in the positive direction

# phi1, phi2, phi3 are q1, q2 and q3

import numpy as np
from config import lengths
from config import servo_angle_limits as lims
from robot.base import State

def check_jacobian():
	"""
	Verify the jacobian computed in robot.base.State is correct, by comparing
	it with the symbolic jacobian of the positions.

	We can't check the positions, because we have nothing to check against.
	"""
	import sympy as sy
	# this is a hack to make np.cos(sympy thing) work properly
	sy.Basic.cos = lambda x: sy.cos(x)
	sy.Basic.sin = lambda x: sy.sin(x)

	# construct the angles and lengths as symbols
	angles = [sy.Symbol(r'\phi_{}'.format(i)) for i in [1, 2, 3]]
	lengths = np.array([sy.Symbol(r'l_{}'.format(i)) for i in [1, 2, 3, 4]])

	# make a state object with them
	s = State(joint_angles=angles).update(lengths=lengths)
	end_effector = s.joint_positions[-1]
	end_jacobian = s.joint_jacobians[-1][:,1:]

	# find the jacobian symbolically and otherwise
	expected = np.asarray(sy.Matrix(end_effector).jacobian(angles))
	actual = end_jacobian

	# compare them
	assert np.all(actual == expected)
	print("Jacobian is:")
	print(repr(actual))


# for this problem we want theta=0 to be the y axis, so just rotate things
rot90 = np.array([
	[0, -1],
	[1, 0]
])

def pJ(qq):
	# we want the jacobian of the last link, wrt all but the first link angle
	J = rot90 @ State(joint_angles=qq).joint_jacobians[-1][:,1:]
	return np.linalg.pinv(J)

def f(qq):
	return rot90 @ State(joint_angles=qq).joint_positions[-1]

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
	check_jacobian()

	q = get_servo_angles([0.3,0.3])
	np.testing.assert_allclose(f(q), [0.3, 0.3], rtol=0.001)


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
