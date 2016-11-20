"""
Measuring the parameters needed to find the relationship between mass distribution and torque
"""
import time

import numpy as np

import ui
from robot import Robot
# Uncomment the following for simulation
#from robot import SimulatedRobot as Robot

with Robot.connect() as r, ui.basic(r) as gui:
    for i in range(3):
    	angles = np.linspace(-90,90,500)

    	for a in angles:
    		r.servo_angle = np.radians([0,0,a])
    		time.sleep(0.02)

    	angles = angles[::-1]

    	for a in angles:
    		r.servo_angle = np.radians([0,0,a])
    		time.sleep(0.02)


        # t = time.time()
        # f = 0.5  # Hz
        # A = np.radians(45)
        # r.servo_angle = np.array([-A*0.5, 0, A])*np.sin(2*np.pi*f*t)
        # print(r.adc_reading)
        # time.sleep(0.01)
