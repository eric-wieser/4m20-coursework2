import time
import tkinter as tk

import numpy as np

from robot import Robot
import ui
from scipy.interpolate import interp1d
#from robot import SimulatedRobot as Robot

SPEED = 0.4

gaitsequence = np.loadtxt('onestep.csv',delimiter=',')
gaitsequence = np.concatenate((gaitsequence, gaitsequence[0:1]))
dts = gaitsequence[:,0] * SPEED


angles = gaitsequence[:,1:]
period = np.sum(dts[:-1])
times = np.zeros(len(angles))
times[1:] = np.cumsum(dts[:-1])

is_first = True

# append the last one to the start
N = len(gaitsequence)
t=np.arange(N)

tinterp = np.arange((N-1) * 100) / 100

angle_for = interp1d(times, angles.T)

with Robot.connect() as r, ui.basic(r) as gui:
    r.target_joint_angle = angles[0]
    time.sleep(2)
    start_time = time.time()
    while True:
        t = (time.time() - start_time) % period
        r.target_joint_angle = np.radians(angle_for(t))
        time.sleep(0.02)
        if not gui.open:
            raise SystemExit
