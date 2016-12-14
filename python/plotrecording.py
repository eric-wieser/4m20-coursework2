#! python3
"""
Pass this program a filename as the first argument
"""
import pickle
import numpy as np
import matplotlib.pyplot as plt
from robot.base import State

import config

import sys

if len(sys.argv) > 1:
	fname = sys.argv[1]
else:
	fname = 'walking.pickle'

with open(fname, 'rb') as f:
	data = pickle.load(f)

error = data.servo - data.actual
error_bounds = config.error_active_lim

in_bounds = (error_bounds[:,0] < error) & (error < error_bounds[:,1])

def stretch_over(data, is_valid, axis=0):# normalize the inputs to match the question examples
    data = np.asarray(data)

    # flat array of the data values
    data_flat = data.ravel()

    # array of indices such that data_flat[indices] == data
    indices = np.arange(data.size).reshape(data.shape)

    # thanks to benjamin here
    stretched_indices = np.maximum.accumulate(is_valid*indices, axis=axis)
    return data_flat[stretched_indices]

actual_oob = np.where(in_bounds, np.nan, data.actual)
servo_oob = np.where(in_bounds, np.nan, data.servo)
data.actual = stretch_over(data.actual, in_bounds)
data.servo = stretch_over(data.servo, in_bounds)

fig, axs = plt.subplots(3, sharex=True)
fig.suptitle('Time-angle plots: {}'.format(fname))
for i, ax in enumerate(axs):
	ax.plot(data.t, np.degrees(data.target[:,i]), label='target')
	l, = ax.plot(data.t, np.degrees(data.actual[:,i]), label='actual')
	ax.plot(data.t, np.degrees(actual_oob[:,i]), color=l.get_color(), alpha=0.5)
	l, = ax.plot(data.t, np.degrees(data.servo[:,i]), label='servo')
	ax.plot(data.t, np.degrees(servo_oob[:,i]), color=l.get_color(), alpha=0.5)
	l, = ax.plot(data.t, np.degrees(error[:,i]), label='displacement')
	ax.axhline(y=np.degrees(error_bounds[i,0]), color=l.get_color(), alpha=0.5)
	ax.axhline(y=np.degrees(error_bounds[i,1]), color=l.get_color(), alpha=0.5)
	ax.grid()
	ax.set(xlabel='time / s', ylabel='$q_{}$ / degrees'.format(i+1))
ax.legend()


target_states = np.stack([State(target).joint_positions for target in data.target], axis=1)
actual_states = np.stack([State(actual).joint_positions for actual in data.actual], axis=1)

fig, ax = plt.subplots()
fig.suptitle('Space-plots: {}'.format(fname))
for targets, actuals in zip(target_states, actual_states):
	l, = ax.plot(targets[:,0],targets[:,1], alpha=0.5, linewidth=3, label='target')
	ax.plot(actuals[:,0],actuals[:,1], color=l.get_color(), label='actual')
ax.legend()
ax.axis('equal')
ax.grid()
plt.show()
