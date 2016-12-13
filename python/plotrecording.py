"""
Pass this program a filename as the first argument
"""
import pickle
import numpy as np
import matplotlib.pyplot as plt
from robot.base import State

import sys

if len(sys.argv) > 1:
	fname = sys.argv[1]
else:
	fname = 'walking.pickle'

with open(fname, 'rb') as f:
	data = pickle.load(f)

fig, axs = plt.subplots(3, sharex=True)
for i in range(3):
	axs[i].plot(data.t, np.degrees(data.target[:,i]), label='target')
	axs[i].plot(data.t, np.degrees(data.actual[:,i]), label='actual')
	axs[i].plot(data.t, np.degrees(data.servo[:,i]), label='servo')
	axs[i].legend()
	axs[i].grid()
	axs[i].set(xlabel='time / s', ylabel='$q_{}$ / degrees'.format(i+1))


target_states = np.stack([State(target).joint_positions for target in data.target], axis=1)
actual_states = np.stack([State(actual).joint_positions for actual in data.actual], axis=1)

print(target_states.shape)

fig, ax = plt.subplots()
for targets, actuals in zip(target_states, actual_states):
	l, = ax.plot(targets[:,0],targets[:,1], alpha=0.5, linewidth=3, label='target')
	ax.plot(actuals[:,0],actuals[:,1], color=l.get_color(), label='actual')
ax.legend()
ax.axis('equal')
ax.grid()
plt.show()
