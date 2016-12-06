import time
import pickle

import numpy as np

class Logger:
    """
    used for logging time-series data about a robot

        l = Logger(robot)
        ...
        l.update()
        ...
        l.dump('outfile.pickle'

    """
    dtype = np.dtype([
        ('t', np.float64),
        ('target', np.float64, 3),
        ('actual', np.float64, 3),
        ('servo', np.float64, 3)
    ])

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
