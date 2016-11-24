import time
import tkinter as tk

import numpy as np

from robot import Robot
import ui
from scipy.interpolate import interp1d
#from robot import SimulatedRobot as Robot

gaitsequence=np.loadtxt('onestep.csv',delimiter=',')
is_first = True

gaitsequence = np.concatenate((gaitsequence, gaitsequence[0:1]))
N = len(gaitsequence)
t=np.arange(N)

tinterp = np.arange((N-1) * 100) / 100

gaitsequence = interp1d(t, gaitsequence.T)(tinterp).T
with Robot.connect() as r, ui.basic(r) as gui:
    while True:
        for i,row in enumerate(gaitsequence):
            if np.isnan(row).any():
                time.sleep(2)
                continue
            r.target_joint_angle = np.radians(row)
            print(row)
            print(i)
            time.sleep(5 if is_first else 0.5 / 100)
            is_first = False
            if not gui.open:
                raise SystemExit
