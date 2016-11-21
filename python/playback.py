import time
import tkinter as tk

import numpy as np

from robot import Robot
import ui
#from robot import SimulatedRobot as Robot

gaitsequence=np.loadtxt('onestep.csv',delimiter=',')
with Robot.connect() as r, ui.basic(r) as gui:
    for row in gaitsequence:
        r.target_joint_angle = np.radians(row)
        print(row)
        time.sleep(2)