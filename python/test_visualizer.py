"""
Demo moving the end link back and forth sinusoidally
"""
import time
import tkinter as tk

import numpy as np

from ui import GeometryVisualizer, BackgroundTK
from robot import SimulatedRobot as Robot

@BackgroundTK
def ui(root):
    v = GeometryVisualizer(root, robot=r, width=400, height=400)
    v.pack(fill=tk.BOTH)

with Robot.connect() as r, ui:
    i = 0
    while ui.open:
        r.servo_angle = np.radians([30, -45, 15])*i/10
        time.sleep(.1)
        i += 1
