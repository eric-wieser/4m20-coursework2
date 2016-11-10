import contextlib
import threading

import serial.threaded
from cobs import cobs
import numpy as np

import messages
import config
import traceback

class Robot(serial.threaded.Packetizer):

    @classmethod
    @contextlib.contextmanager
    def connect(cls, port):
        """
        Create a new robot, given a port

        This spins off a background thread to read the serial link, and ensures
        that the connection is functional by sending some pings.

        This must be used in a with statement:

            with Robot.connect('COM4') as r:
                r.servo_angle = [0, 0, 0]
        """
        conn = serial.Serial(port=port, baudrate=115200)
        with serial.threaded.ReaderThread(conn, cls) as r:
            r.ping()
            r.ping()
            r.config(servo_limits_us=config.servo_limits)
            yield r

    def __init__(self):
        """ Create a new robot. This is called internally by ReaderThread """
        super().__init__()
        self._servo_us = None
        self._adc_reading = None
        self._ping_recvd = False

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
        self._write_message(messages.ServoPulse(value))
        self._servo_us = np.array(value, dtype=np.uint16)

    @property
    def adc_reading(self):
        """ the potentiometer readings, 0 - 1023 """
        while self._adc_reading is None:
            pass
        return self._adc_reading

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

if __name__ == '__main__':
    import time
    with Robot.connect('COM4') as r:
        r.servo_angle = np.array([0, 0, 0])
        time.sleep(10)
        r.servo_angle = None
