"""
Demo moving the end link back and forth sinusoidally
"""
import time

import numpy as np

from robot import Robot

with Robot.connect() as r:
    while True:
        t = time.time()
        f = 0.5  # Hz
        A = np.radians(45)
        r.servo_angle = np.array([0, 0, A*np.sin(2*np.pi*f*t)])
