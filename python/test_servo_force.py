"""
Demo a compliance controller exerting zero torque
"""
import time

import numpy as np

from robot import Robot

with Robot.connect() as r:
    time.sleep(1)

    tot = np.uint32(0)
    for i in range(10):
        adc = r.adc_reading
        tot += adc
        time.sleep(.05)
    tot //= 10

    print("Enabling compliance control, {}".format(tot))

    r.target_adc_reading = tot
    time.sleep(100)
