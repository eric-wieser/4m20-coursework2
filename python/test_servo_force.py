"""
Demo a compliance controller exerting zero torque
"""
import time
import tkinter as tk

import numpy as np

from robot import Robot
import config
from ui import GeometryVisualizer, BackgroundTK

@BackgroundTK
def ui(root):
    v = GeometryVisualizer(root, robot=r, width=400, height=400)
    v.pack(fill=tk.BOTH)

with Robot.connect() as r, ui:
    r.target_adc_reading = config.adc_0

    while ui.open:
        time.sleep(0.1)
