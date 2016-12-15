import time
import pickle

import numpy as np

import config

basic_fields = [
    ('t', np.float64),
    ('target', np.float64, 3),
    ('actual', np.float64, 3),
    ('servo', np.float64, 3)
]
extra_fields = [
    ('error', np.float64, 3),
    ('in_bounds', np.bool, 3),
    ('actual_oob', np.float64, 3),
    ('servo_oob', np.float64, 3)
]
basic_dtype = np.dtype(basic_fields)
full_dtype = np.dtype(basic_fields + extra_fields)

class Logger:
    """
    used for logging time-series data about a robot

        l = Logger(robot)
        ...
        l.update()
        ...
        l.dump('outfile.pickle'

    """
    dtype = basic_dtype

    def __init__(self, robot):
        self._robot = robot
        self._data = []
        self._t0 = time.time()

    def update(self):
        s = self._robot.state
        self._data.append(
            (time.time() - self._t0, self._robot.target_joint_angle, s.joint_angles, s.servo_angle)
        )

    def as_records(self):
        return np.core.records.fromrecords(self._data, dtype=self.dtype)

    def dump(self, fname):
        with open(fname, 'wb') as f:
             pickle.dump(self.as_records(), f)


def stretch_over(data, is_valid, axis=0):
    """
    Takes an array, and a mask where it is valid, and stretches the present
    values forward along `axis` to cover the absent ones. The first value is
    always treated as valid.

    >>> stretch_over([0, 1, 2, 4, 3], [0, 0, 1, 0, 1])
    array([0, 0, 2, 2, 3])
    """
    data = np.asarray(data)
    is_valid = np.asarray(is_valid)

    # flat array of the data values
    data_flat = data.ravel()

    # array of indices such that data_flat[indices] == data
    indices = np.arange(data.size).reshape(data.shape)

    # thanks to benjamin here
    stretched_indices = np.maximum.accumulate(is_valid*indices, axis=axis)
    return data_flat[stretched_indices]


def augment(data):
    """
    After loading data from a log file, pass it through `augment` to calculate:

    * spring displacements
    * torque limits being hit

    And to try and correct estimation issues around torque limits
    """
    full_dtype = np.dtype(basic_fields + extra_fields)

    aug = np.recarray(data.shape, dtype=full_dtype)
    for name in basic_dtype.names:
        setattr(aug, name, getattr(data, name))

    assert np.all(data.servo == aug.servo)

    aug.error = data.servo - data.actual
    error_bounds = config.error_active_lim

    aug.in_bounds = (error_bounds[:,0] < aug.error) & (aug.error < error_bounds[:,1])

    aug.actual_oob = np.where(aug.in_bounds, np.nan, data.actual)
    aug.servo_oob = np.where(aug.in_bounds, np.nan, data.servo)
    aug.actual = stretch_over(data.actual, aug.in_bounds)
    aug.servo = stretch_over(data.servo, aug.in_bounds)

    return aug