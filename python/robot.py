import contextlib
import threading
import warnings
import time
import enum
import traceback
import abc

import serial.threaded
from cobs import cobs
import numpy as np

import messages
import config

def _find_arduino_port() -> str:
    import serial.tools.list_ports
    for pinfo in serial.tools.list_ports.comports():
        if pinfo.serial_number == '85430353531351B09121':
            return pinfo.device

    raise IOError("Could not find an arduino - is it plugged in?")


class RobotBase(metaclass=abc.ABCMeta):
    @classmethod
    @abc.abstractmethod
    def connect(cls): pass

    @property
    @abc.abstractmethod
    def servo_angle(self): raise NotImplementedError
    @servo_angle.setter
    def servo_angle(self, value): raise NotImplementedError

    @property
    @abc.abstractmethod
    def adc_reading(self): raise NotImplementedError

    @property
    @abc.abstractmethod
    def target_adc_reading(self, value): raise NotImplementedError

    @property
    def link_angles(self):
        res = np.empty(4)
        res[0] = 0 # first link is clamped
        res[1:] = np.cumsum(self.joint_angles)
        return res

    @property
    def joint_angles(self):
        s_angle = 0
        if self.servo_angle is not None:
            s_angle += self.servo_angle
        if self.adc_reading is not None:
            s_angle -= self.angle_error
        return s_angle

    @property
    def joint_positions(self):
        angles = self.link_angles
        directions = np.stack([
            np.cos(angles),
            np.sin(angles)
        ], axis=1)
        return np.add.accumulate(
            config.lengths[:,np.newaxis]*directions
        )

    @property
    def joints_stalled(self):
        return (self.adc_reading >= 680) | (self.adc_reading <= 360)

    @property
    def angle_error(self):
        return (self.adc_reading - config.adc_0)*config.rad_per_adc


class ControlMode(enum.Enum):
    Period = object()
    Torque = object()

class ArduinoRobot(RobotBase, serial.threaded.Packetizer):
    """ The low-level messaging-level operations of the robot """

    @classmethod
    @contextlib.contextmanager
    def connect(cls, port=None):
        """
        Create a new robot, given a port

        This spins off a background thread to read the serial link, and ensures
        that the connection is functional by sending some pings.

        This must be used in a with statement:

            with Robot.connect('COM4') as r:
                r.servo_angle = [0, 0, 0]
        """
        if port is None:
            port = _find_arduino_port()
        else:
            warnings.warn("Choosing a specific port may make this only work on your machine!")

        conn = serial.Serial(port=port, baudrate=115200)
        with serial.threaded.ReaderThread(conn, cls) as r:
            time.sleep(0.5)
            r.ping()
            r.ping()
            r.config(
                servo_limits_us=config.servo_limits,
                adc_zero=config.adc_0,
                servo_per_adc=config.servo_per_rad * config.rad_per_adc
            )
            try:
                yield r
            finally:
                r.servo_us = None

    def __init__(self):
        """ Create a new robot. This is called internally by ReaderThread """
        super().__init__()
        self._servo_us = None
        self._adc_reading = None
        self._ping_recvd = False
        self._mode = ControlMode.Period

    def handle_packet(self, packet: bytes):
        """
        implements Packetizer.handle_packet. Shame this can't be private
        """
        # decode packet
        try:
            raw = cobs.decode(packet)
            msg = messages.Message.deserialize(raw)
        except (messages.DecodeError, cobs.DecodeError) as e:
            # not much we can do about garbage messages, so log and continue
            print(traceback.format_exception_only(type(e), e)[0].strip())
            return

        # update state, depending on type of message
        if isinstance(msg, messages.Sensor):
            self._adc_reading = msg
        elif isinstance(msg, messages.IMUScaled):
            pass
        elif isinstance(msg, messages.ServoPulse):
            if self._mode == ControlMode.Torque:
                self._servo_us = np.asarray(msg)
        elif isinstance(msg, messages.Ping):
            self._ping_recvd = True

    def _write_message(self, message: messages.Message):
        """ Write a message to the serial link """
        raw = message.serialize()
        encoded = cobs.encode(raw)
        self.transport.write(encoded + b'\x00')


    def ping(self):
        """
        Send a bunch of pings, and wait for a response to any of them.

        This is a bad way of testing when the arduino is ready
        """
        self._ping_recvd = False
        for i in range(100):
            self._write_message(messages.Ping())
            if self._ping_recvd:
                return
            time.sleep(0.05)

        raise IOError('Never got a response ping')


    def config(self, *, servo_limits_us, adc_zero, servo_per_adc):
        msg = messages.JointConfig(
            tuple(servo_limits_us) + tuple(adc_zero) + tuple(servo_per_adc)
        )
        #TODO: verify this went through!
        self._write_message(msg)
        self._write_message(msg)

    # servo control
    @property
    def servo_us(self):
        """
        the pulse widths of the servos, in us
        `None` means the servo is off.
        """
        return self._servo_us

    @servo_us.setter
    def servo_us(self, value):
        if value is None:
            value = (0xffff,)*3
        self._mode = ControlMode.Period
        self._write_message(messages.ServoPulse(value))
        self._servo_us = np.clip(np.array(value, dtype=np.uint16), *config.servo_limits)

    @property
    def servo_angle(self):
        """ return the servo angle in radians """
        if self.servo_us is None:
            return None
        return (self.servo_us - config.servo_0) / config.servo_per_radian
    @servo_angle.setter
    def servo_angle(self, value):
        if value is not None:
            value = value * config.servo_per_radian + config.servo_0
            value = value.astype(np.uint16)
        self.servo_us = value


    @property
    def adc_reading(self):
        """ the potentiometer readings, 0 - 1023 """
        while self._adc_reading is None:
            pass
        return np.asarray(self._adc_reading)

    @property
    def target_adc_reading(self): raise ValueError
    @target_adc_reading.setter
    def target_adc_reading(self, value):
        """ use force control to try and hit the desired ADC value """
        value = value.astype(np.uint16)
        self._mode = ControlMode.Torque
        self._write_message(messages.ServoForce(value))


class SimulatedRobot(RobotBase):
    """
    A simulated version of the robot that requires no connected hardware
    """
    @classmethod
    @contextlib.contextmanager
    def connect(cls):
        yield cls()

    def __init__(self):
        self._adc_reading = np.ones(3) * 512
        self._servo_angle = np.zeros(3)

    @property
    def servo_angle(self):
        return self._servo_angle

    @servo_angle.setter
    def servo_angle(self, value):
        self._servo_angle = value

    @property
    def adc_reading(self):
        return self._adc_reading


    @RobotBase.angle_error.setter
    def angle_error(self, value):
        """ for debugging """
        self._adc_reading = value / config.rad_per_adc + config.adc_0

    @property
    def target_adc_reading(self): raise NotImplementedError
    @target_adc_reading.setter
    def target_adc_reading(self, value): raise NotImplementedError


Robot = ArduinoRobot
