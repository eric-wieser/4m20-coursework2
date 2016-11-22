"""
Demo moving the end link back and forth sinusoidally
"""
import time

import numpy as np

import ui
from robot import Robot
# Uncomment the following for simulation
#from robot import SimulatedRobot as Robot

p_out = np.array([80, 20, 80])
p_in = np.array([45, 90, 45])

left_step = np.array([30, 0, -30])
right_step = np.array([-30, 0, 30])

def sine_ease(t):
    return 0.5 - np.cos(np.pi*t)*0.5

def lerp(a, b, f):
    return a*(1 - f) + b*f

PERIOD = 5

with Robot.connect() as r, ui.basic(r) as gui:
    while gui.open:
        t = time.time()
        t = (t % PERIOD) / PERIOD
        t = t * 4

        if t < 1:
            degrees = lerp(p_in, p_out, sine_ease(t)) + left_step
        elif t < 2:
            degrees = p_out + lerp(left_step, right_step, sine_ease(t - 1))
        elif t < 3:
            degrees = lerp(p_out, p_in, sine_ease(t - 2)) + right_step
        else:
            degrees = p_in + lerp(right_step, left_step, sine_ease(t - 3))

        r.target_joint_angle = np.radians(degrees)
        time.sleep(0.01)
