#! python3
"""
Pass this program a filename as the first argument
"""
import sys
import pickle

import numpy as np
import matplotlib.pyplot as plt

from robot.base import State
import config
from logger import augment

def add_arrow(line, direction='right', size=15, color=None, n=1):
    """
    add an arrow to a line.

    line:       Line2D object
    position:   x-position of the arrow. If None, mean of xdata is taken
    direction:  'left' or 'right'
    size:       size of the arrow in fontsize points
    color:      if None, line color is taken.
    """
    if color is None:
        color = line.get_color()

    xdata = line.get_xdata()
    ydata = line.get_ydata()

    # find closest index
    dists = np.cumsum(np.hypot(np.diff(xdata), np.diff(ydata)))
    dists = np.concatenate(([0], dists))
    total_dist = dists[-1]


    for i in range(1, n+1):
        target_dist = total_dist * i / (n+1)
        end_ind = np.argmax(dists >= target_dist)
        start_ind = end_ind - 1

        frac = (target_dist - dists[start_ind]) / (dists[end_ind] - dists[start_ind])
        end_x = xdata[start_ind]*(1-frac) + xdata[end_ind]*frac
        end_y = ydata[start_ind]*(1-frac) + ydata[end_ind]*frac

        line.axes.annotate('',
            xytext=(xdata[start_ind], ydata[start_ind]),
            xy=(end_x, end_y),
            arrowprops=dict(arrowstyle="->", color=color),
            size=size
        )

if len(sys.argv) > 1:
	fname = sys.argv[1]
else:
	fname = 'walking.pickle'

with open(fname, 'rb') as f:
	data = pickle.load(f)

# add all the extra calculated fields
aug = augment(data)

error_bounds = config.error_active_lim

fig, axs = plt.subplots(3, sharex=True)
fig.patch.set(alpha=0)
fig.suptitle('Time-angle plots: {}'.format(fname))
for i, ax in enumerate(axs):
	ax.plot(aug.t, np.degrees(aug.target[:,i]), label='target')
	l, = ax.plot(aug.t, np.degrees(aug.actual[:,i]), label='actual')
	ax.plot(aug.t, np.degrees(aug.actual_oob[:,i]), color=l.get_color(), alpha=0.5)
	l, = ax.plot(aug.t, np.degrees(aug.servo[:,i]), label='servo')
	ax.plot(aug.t, np.degrees(aug.servo_oob[:,i]), color=l.get_color(), alpha=0.5)
	l, = ax.plot(aug.t, np.degrees(aug.error[:,i]), label='displacement')
	ax.axhline(y=np.degrees(error_bounds[i,0]), color=l.get_color(), alpha=0.5)
	ax.axhline(y=np.degrees(error_bounds[i,1]), color=l.get_color(), alpha=0.5)
	ax.grid()
	ax.set(xlabel='time / s', ylabel='$q_{}$ / degrees'.format(i+1))
ax.legend()


target_states = np.stack([State(target).joint_positions for target in aug.target], axis=1)
actual_states = np.stack([State(actual).joint_positions for actual in aug.actual], axis=1)

fig, ax = plt.subplots()
fig.patch.set(alpha=0)
fig.suptitle('Space-plots: {}'.format(fname))
for targets, actuals in zip(target_states, actual_states):
	l, = ax.plot(targets[:,0],targets[:,1], alpha=0.5, linewidth=3, label='target')
	ax.plot(actuals[:,0],actuals[:,1], color=l.get_color(), label='actual')
ax.legend()
ax.axis('equal')
ax.grid()


# this data is super noisy, so filter it
N = 11
filt = np.ones(N) / N

fig, axs = plt.subplots(3)
fig.patch.set(alpha=0)
fig.suptitle('Angle-force plots: {}'.format(fname))
for i, ax in enumerate(axs):
	# extract aug, and smooth it
	actual_i = aug.actual[:,i]
	error_i = aug.error[:,i]
	sm_actual_i = np.convolve(actual_i, filt)[N//2:][:len(actual_i)]
	sm_error_i = np.convolve(error_i, filt)[N//2:][:len(error_i)]

	# plot both
	l, = ax.plot(np.degrees(actual_i), np.degrees(error_i), alpha=0.25)
	l, = ax.plot(np.degrees(sm_actual_i), np.degrees(sm_error_i), color=l.get_color())
	add_arrow(l, n=5)

	ax.grid()
	ax.set(xlabel=r'$\phi_{}$ / degrees', ylabel='spring angle / degrees'.format(i+1))

plt.show()
