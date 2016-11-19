"""
Demo moving the end link back and forth sinusoidally
"""

import copy

class PathVertex():
	def __init__(self, x, y, t):
		self.x = x
		self.y = y
		self.t = t

def forwardDifferenceVertices( vertices ):
	velocityPath = []
	for vIndex,thisVertex in enumerate(vertices):
		if vIndex == len(vertices)-1:
			velocityPath.append(PathVertex(0,0,thisVertex.t))
		else:
			velocityPath.append(PathVertex(vertices[vIndex+1].x-thisVertex.x , vertices[vIndex+1].y-thisVertex.y, thisVertex.t))
	return velocityPath

class Path():
	def __init__(self, endVertices):
		self.calculateLimbVertices(endVertices)
		self.calculateJointAngles()
		self.calculateJointVelocities()

	def calculateLimbVertices(self,endVertices):
		self.vertices[0] = copy.deepcopy(endVertices)

	def calculateJointAngles(self):
		pass

	def calculateJointAngleVelocity(self):
		pass

	def calculateJointVelocities(self):
		self.endVelocity = forwardDifferenceVertices(self.endVertices)




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
