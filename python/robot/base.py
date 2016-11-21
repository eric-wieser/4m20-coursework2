import abc
import numpy as np

import config

class Robot(metaclass=abc.ABCMeta):
    @classmethod
    @abc.abstractmethod
    def connect(cls): pass

    @property
    @abc.abstractmethod
    def state(self): raise NotImplementedError

    # shorthand for state properties
    @property
    def adc_reading(self): return self.state.adc_reading
    @property
    def servo_angle(self): return self.state.servo_angle
    @property
    def link_angles(self): return self.state.link_angles
    @property
    def joint_angles(self): return self.state.joint_angles
    @property
    def joint_positions(self): return self.state.joint_positions
    @property
    def joints_stalled(self): return self.state.joints_stalled
    @property
    def angle_error(self): return self.state.angle_error

    # setters
    @servo_angle.setter
    @abc.abstractmethod
    def servo_angle(self, value): raise NotImplementedError

    @property
    @abc.abstractmethod
    def target_adc_reading(self, value): raise NotImplementedError


class State(metaclass=abc.ABCMeta):
    def update(self, **kwargs):
        """
        Convenience method to update a property of the state and return the modified value

        s.update(servo_us=[0,0,0])
        """
        for k, v in kwargs.items():
            if hasattr(self, k):
                setattr(self, k, v)
            else:
                raise AttributeError
        return self

    @property
    @abc.abstractmethod
    def servo_angle(self):
        """ Angle of each servo, in radians """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def adc_reading(self):
        """ Reading from the potentiometer in arbritary units """
        raise NotImplementedError

    @property
    def joints_stalled(self):
        """ Boolean array, of whether the joint is at maximum torque """
        return (self.adc_reading >= 680) | (self.adc_reading <= 360)

    @property
    def angle_error(self):
        """ Difference in angle between the joint and the servo, calculated from the force """
        return (self.adc_reading - config.adc_0)*config.rad_per_adc

    @property
    def joint_angles(self):
        """ Overall joint angle between two adjacent links """
        return self.servo_angle - self.angle_error

    @property
    def link_angles(self):
        """ The angles of the links, measured relative to link 0 """
        res = np.empty(4)
        res[0] = 0 # first link is clamped
        res[1:] = np.cumsum(self.joint_angles)
        return res

    @property
    def joint_positions(self):
        """ The positions of the joints, a nx2 array """
        angles = self.link_angles
        directions = np.stack([
            np.cos(angles),
            np.sin(angles)
        ], axis=1)
        return np.add.accumulate(
            config.lengths[:,np.newaxis]*directions
        )