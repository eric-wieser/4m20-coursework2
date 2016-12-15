import tkinter as tk
from tkinter import ttk
from contextlib import contextmanager

import config
import time
from ui import GeometryVisualizer
import numpy as np

@contextmanager
def frame(parent, text=""):
    if text:
        f = ttk.LabelFrame(parent, text=text)
    else:
        f = ttk.Frame(parent)
    yield f

class SliderGui:
    """
    A GUI that shows a set of sliders to control the motors and read the sensors
    """
    def __init__(self, start=[1500] * config.N):
        # create the window
        root = tk.Tk()
        root.resizable(True, True)
        root.geometry('400x450')

        self._pot_sliders = []
        self._root = root
        self._servo_values = np.asarray(start)

        controls = ttk.Frame(root)

        # create the servo sliders
        with frame(controls, text="Servo angles") as servo_sliders:
            vars = [tk.DoubleVar() for i in range(config.N)]
            for i in range(config.N):
                scale = tk.Scale(servo_sliders,
                    from_=np.degrees(config.servo_angle_limits[i,0]), to=np.degrees(config.servo_angle_limits[i,1]),
                    tickinterval=30, orient=tk.HORIZONTAL, takefocus=1,
                    resolution=0.5,
                    variable = vars[i],
                    command=lambda evt, i=i: self._servo_changed(evt, i))
                scale.pack(fill=tk.BOTH)
                scale.set(self._servo_values[i])

        # create the potentiometer sliders
        with frame(controls, text="Analog sensor readings") as pot_sliders:
            for i in range(config.N):
                scale = tk.Scale(pot_sliders, from_=0, to=1024, tickinterval=128,
                    # state=tk.DISABLED,
                    orient=tk.HORIZONTAL)
                scale.pack(fill=tk.BOTH)
                self._pot_sliders.append(scale)

        # add manual entry fields
        with frame(controls, text="Servo angles") as servo_fields:
            for v in vars:
                e = ttk.Entry(servo_fields, textvariable=v, width=6)
                e.pack(fill=tk.BOTH, side=tk.LEFT, padx=1, pady=1)
            # go button
            def button_pressed():
                self._servo_values[:] = [v.get() for v in vars]
                self._updated()
            b = ttk.Button(servo_fields, text='Go!', command=button_pressed)
            b.pack(fill=tk.BOTH, side=tk.LEFT)


        # position feedback toggle
        self.feedback_v = tk.IntVar()
        feedback_button = ttk.Checkbutton(controls, text="Use position feedback", variable=self.feedback_v,command=lambda : self._updated())

        # lay out the large components
        servo_fields.pack(padx=10, pady=10, side=tk.BOTTOM, expand=True)
        feedback_button.pack(side=tk.BOTTOM)
        servo_sliders.pack(padx=10, pady=10, side=tk.LEFT, fill=tk.BOTH, expand=True)
        pot_sliders.pack(padx=10, pady=10, side=tk.LEFT, fill=tk.BOTH, expand=True)

        controls.pack(fill=tk.BOTH, side=tk.BOTTOM, expand=True)

        self._on_servo_changed = lambda values,use_feedback: None

    def _servo_changed(self, val, i):
        """ called when a single slider value changes"""
        self._servo_values[i] = float(val)
        self._updated()

    def _updated(self):
        self.on_servo_changed(self._servo_values, self.feedback_v.get())

    def _update_ui(self, pot_readings):
        """ Update the ui with the given potentiometer readings """
        for scale, value in zip(self._pot_sliders, pot_readings):
            scale['state'] = tk.NORMAL
            scale.set(value)
            scale['state'] = tk.DISABLED

    def update_ui(self, pot_readings):
        self._root.after(10, lambda: self._update_ui(pot_readings))

    # replace this with a function to call with all the servo values
    @property
    def on_servo_changed(self):
        return self._on_servo_changed
    @on_servo_changed.setter
    def on_servo_changed(self, callback):
        self._on_servo_changed = callback
        self._updated()


if __name__ == '__main__':
    from robot import Robot
    import messages

    # Run a simple test of sending a packet, and getting some responses
    with Robot.connect() as robot:
        gui = SliderGui(start=(60,60,60))
        vis = GeometryVisualizer(gui._root, robot=robot, width=500, height=500)
        vis.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        def send_it(v, feedback):
            if feedback:
                robot.target_joint_angle = np.radians(v)
            else:
                robot.servo_angle = np.radians(v)
        gui.on_servo_changed = send_it

        # we have to mess around here with .after to keep the UI thread responsive
        def update_it():
            gui.update_ui(robot.adc_reading)
            gui._root.after(10, update_it)
        gui._root.after(0, update_it)

        gui._root.mainloop()

        # turn off the servos when we're done
        robot.servo_us = -1
