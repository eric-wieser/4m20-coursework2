import time
import tkinter as tk

import numpy as np
from scipy.interpolate import interp1d

from robot import Robot
#from robot import SimulatedRobot as Robot
from logger import Logger
import ui
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
    logger = Logger(r)
    start_time = time.time()
    while True:
        t = time.time() - start_time
        t_offset = t % period
        target = np.radians(angle_for(t_offset))

        r.target_joint_angle = target
        time.sleep(0.02)
        logger.update()
        if not gui.open:
            break

logger.dump('recording.pickle')
