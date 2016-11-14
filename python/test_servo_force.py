"""
Demo a compliance controller exerting zero torque
"""
import time
import tkinter as tk

import numpy as np

from robot import Robot
import config
import ui

with Robot.connect() as r, ui.basic(r)
    r.target_adc_reading = config.adc_0

    while ui.open:
        time.sleep(0.1)
