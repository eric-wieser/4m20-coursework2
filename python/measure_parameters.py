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


    # angles = np.linspace(-90,90,11)
    # angles = np.concatenate((angles, angles[::-1]))
    # angles = np.tile(angles,3)

    # with open('sausages.tsv','w') as f:
    #     for a in angles:
    #         r.servo_angle = np.radians([-45,-45,a])
    #         time.sleep(0.5)
    #         print(r.joint_angles[2], r.angle_error[2], sep='\t', file=f)


    angles_pos = np.stack((
        np.linspace(-90,0,51),
        np.linspace(0,-90,51)
    ), axis=-1)
   
    angles_0 = np.stack((
        np.linspace(0,0,91),
        np.linspace(-90,90,91)
    ), axis=-1)

    angles_neg = np.stack((
        np.linspace(0,90,51),
        np.linspace(90,0,51)
    ), axis=-1)

    angles = np.concatenate((angles_pos, angles_0, angles_neg))
    angles = np.concatenate((angles, angles[::-1]))
    angles = np.tile(angles,(3,1))
    print(angles)

    with open('moresausages.tsv','w') as f:
        for i,(a,b) in enumerate(angles):
            r.servo_angle = np.radians([45,a,b])
            time.sleep(0.05)
            if i% 10 == 0:
                time.sleep(0.25)
                print(r.joint_angles[1], r.angle_error[1], r.joint_angles[2], r.angle_error[2], sep='\t', file=f)

    # turned upside down (relative to first test) to avoid wire interference


    # angles = np.linspace(-90,90,11)
    # angles = np.concatenate((angles, angles[::-1]))
    # angles = np.tile(angles,3)

    # with open('sausages.tsv','w') as f:
    #     for a in angles:
    #         r.servo_angle = np.radians([-45,-45,a])
    #         time.sleep(0.5)
    #         print(r.servo_angle[2], r.angle_error[2], sep='\t', file=f)

        # t = time.time()
        # f = 0.5  # Hz
        # A = np.radians(45)
        # r.servo_angle = np.array([-A*0.5, 0, A])*np.sin(2*np.pi*f*t)
        # print(r.adc_reading)
        # time.sleep(0.01)
