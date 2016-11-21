import contextlib
import enum
import time
import traceback
import warnings

from cobs import cobs
import numpy as np
import serial
import serial.threaded

from . import base
import config
import messages

def _find_arduino_port() -> str:
    import serial.tools.list_ports
    for pinfo in serial.tools.list_ports.comports():
        if pinfo.serial_number == '85430353531351B09121':
            return pinfo.device

    raise IOError("Could not find an arduino - is it plugged in?")


class ControlMode(enum.Enum):
    Period = object()
    Torque = object()
    Position = object()


class State(base.State):
    def __init__(self, servo_us, adc_reading):
        self._adc_reading = np.empty(3, np.uint16)
        self._servo_us = np.empty(3, np.uint16)

        self._servo_us[:] = servo_us
        self._adc_reading[:] = adc_reading

    @base.State.adc_reading.getter
    def adc_reading(self):
        return self._adc_reading

    @property
    def servo_us(self):
        """
        the pulse widths of the servos, in us
        `None` means the servo is off.
        """
        return self._servo_us
    @servo_us.setter
    def servo_us(self, value):
        value = np.asarray(value, dtype=np.uint16)
        clipped = np.clip(value, *config.servo_limits)
        self._servo_us[...] = np.where(value == np.uint16(-1), -1, clipped)

    # servo microsseconds to radians
    @base.State.servo_angle.getter
    def servo_angle(self):
        """ return the servo angle in radians """
        angle = (self.servo_us - config.servo_0) / config.servo_per_radian
        return np.where(self.servo_us == np.uint16(-1), np.nan, angle)
    @servo_angle.setter
    def servo_angle(self, value):
        us = value * config.servo_per_radian + config.servo_0
        self.servo_us = np.where(np.isnan(value), -1, us.astype(np.uint16))


class Robot(base.Robot, serial.threaded.Packetizer):
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
                servo_per_adc=config.servo_per_radian * config.rad_per_adc
            )
            try:
                yield r
            finally:
                r.servo_us = -1

    def __init__(self):
        """ Create a new robot. This is called internally by ReaderThread """
        super().__init__()
        self._servo_us = np.full(3, fill_value=-1, dtype=np.uint16)
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
            print(traceback.format_exception_only(type(e), e)[0].strip(), repr(packet))
            return

        # update state, depending on type of message
        if isinstance(msg, messages.Sensor):
            self._adc_reading = msg
        elif isinstance(msg, messages.IMUScaled):
            pass
        elif isinstance(msg, messages.ServoPulse):
            if self._mode in (ControlMode.Torque, ControlMode.Position):
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
            tuple(servo_limits_us) + tuple(adc_zero.astype(np.uint16)) + tuple(servo_per_adc)
        )
        #TODO: verify this went through!
        self._write_message(msg)
        self._write_message(msg)

    def _write_us(self, value):
        self._servo_us = value
        self._mode = ControlMode.Period
        self._write_message(messages.ServoPulse(value))
        self._servo_us = np.clip(np.array(value, dtype=np.uint16), *config.servo_limits)

    # servo control
    @property
    def state(self):
        while self._adc_reading is None:
            pass
        return State(servo_us=self._servo_us, adc_reading=self._adc_reading)


    @property
    def servo_us(self): return self.state.servo_us
    @servo_us.setter
    def servo_us(self, value):
        self._write_us(self.state.update(servo_us=value).servo_us)

    @property
    def servo_angle(self): return self.state.servo_angle
    @servo_angle.setter
    def servo_angle(self, value):
        self._write_us(self.state.update(servo_angle=value).servo_us)

    @property
    def target_adc_reading(self): raise ValueError
    @target_adc_reading.setter
    def target_adc_reading(self, value):
        """ use force control to try and hit the desired ADC value """
        value = value.astype(np.uint16)
        self._mode = ControlMode.Torque
        self._write_message(messages.ServoForce(value))

    @property
    def target_servo_position(self): raise ValueError
    @target_servo_position.setter
    def target_servo_position(self, value):
        """ use position control to try and hit the desired angle """
        value = value.astype(np.uint16)
        self._mode = ControlMode.Position
        self._write_message(messages.ServoPosition(value))
