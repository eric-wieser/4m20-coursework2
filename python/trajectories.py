import numpy as np

import invkin
from config import servo_angle_limits as lims

def heart(center=np.array([-0.329, 0.24])):
	heartr = np.zeros((104, 2))
	i = 0
	for t in np.arange(0.1, 2*np.pi, 0.06):
		x = 16*np.power(np.sin(t),3)
		y = 13*np.cos(t) - 5*np.cos(2*t) - 2*np.cos(3*t) - np.cos(4*t)
		heartr[i] = center + np.array([0.0048*x, 0.007*y])
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

qstart = np.array([0.0, 0.0, 0.0])

listr = np.array([[0.2, 0.2], [0.3, 0.3], [0.35, 0.35], [0.4, 0.35]])

qheart = invkin.list_of_angles(heart(),qstart) # list of angles to draw a heart with discontinuity
qpadding = remove_discontinuity(qheart) # list of angles to get from qheart[-1] back to qheart[0]
qheart = np.concatenate((qheart,qpadding), axis=0) # list of angles to draw a heart with discontinuity removed


if __name__ == '__main__':
	try:
		from matplotlib import pyplot as plt
	except ImportError:
		import warnings
		warnings.warn("Can't plot the heart :(")
	else:
		r = heart()
		fig, ax = plt.subplots()
		ax.plot(r[:,0], r[:,1], label='Intended')
		r2 = np.array([invkin.f(q) for q in qheart])
		ax.plot(r2[:,0], r2[:,1], label='Calculated')
		ax.legend()
		ax.axis('equal')
		plt.show()