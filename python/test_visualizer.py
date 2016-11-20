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
    while ui.open:
        t = time.time()
        f = 0.5  # Hz
        A = np.radians(45)
        r.servo_angle = np.array([-A*0.5, 0, A])*np.sin(2*np.pi*f*t)
        r.angle_error = r.servo_angle / 2
        time.sleep(0.01)
