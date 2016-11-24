import contextlib

import numpy as np

from . import base
import config

class State(base.StateWithSprings):
    pass

class Robot(base.Robot):
    """
    A simulated version of the robot that requires no connected hardware
    """
    @classmethod
    @contextlib.contextmanager
    def connect(cls):
        yield cls()

    def __init__(self):
        self._adc_reading = config.adc_0
        self._servo_angle = np.zeros(3)

    @property
    def state(self):
        return State(adc_reading=self._adc_reading, servo_angle=self._servo_angle)

    @property
    def servo_angle(self):
        return self._servo_angle

    @servo_angle.setter
    def servo_angle(self, value):
        self._servo_angle = value

    @base.Robot.angle_error.setter
    def angle_error(self, value):
        """ for debugging """
        self._adc_reading = value / config.rad_per_adc + config.adc_0

    @property
    def target_adc_reading(self): raise NotImplementedError
    @target_adc_reading.setter
    def target_adc_reading(self, value): raise NotImplementedError
