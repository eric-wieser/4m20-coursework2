"""
Demo moving the end link back and forth sinusoidally
"""
import time

import numpy as np

import ui
from robot import Robot
# Uncomment the following for simulation
from robot import SimulatedRobot as Robot


from invkin import qheart

with Robot.connect() as r:
	gui =  ui.basic(r) 
	@gui.with_
	def _(ui):
	    while gui.open:
		    print('started')
		    for i in range(qheart.shape[0]):
		        print(i)
		        r.servo_angle = qheart[i]
		        print(i)
		        time.sleep(0.1)


