import serial
from cobs import cobs
import messages

from queue import Queue
from threading import Lock, Thread
import select

class Channel:
    """
    A channel for sending Message objects.

    Messages are encoded using COBS, with a trailing null byte added to each packet.
    """
    def __init__(self, port='COM3'):
        self._conn = serial.Serial(port=port, baudrate=115200)
        self._packets = Queue()
        self._running = False

        self._backgroundThread = Thread(target=self._buffer_thread)

    def __enter__(self):
        self._conn.__enter__()
        self._running = True
        self._backgroundThread.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._running = False
        self._backgroundThread.join()
        self._conn.__exit__(exc_type, exc_val, exc_tb)

    def _buffer_thread(self):
        """ this thread runs in the background to process packets """
        buffer = b''
        while self._running:
            # select.select([self._conn], [], [], 0.5)
            avail = self._conn.in_waiting
            if avail:
                buffer += self._conn.read(avail)
                *packets, buffer = buffer.split(b'\x00')
                for p in packets:
                    self._packets.put(p)
                    self.on_message()

    def write(self, message: messages.Message):
        """ Write a message to the channel """
        raw = message.serialize()
        encoded = cobs.encode(raw)
        self._conn.write(encoded + b'\x00')

    def read(self, timeout=None, block=False):
        """ wait for a message to arrive """
        encoded = self._packets.get(timeout=timeout, block=block)
        raw = cobs.decode(encoded)
        return messages.Message.deserialize(raw)

    on_message = lambda x: None


if __name__ == '__main__':
    # Run a simple test of sending a packet, and getting some responses
    with Channel('COM4') as c:
        c.write(messages.Control((1400, 1400, 1400)))

        while True:
            try:
                p = c.read(block=True)
            except (cobs.DecodeError, messages.DecodeError) as e:
                print(e)
            else:
                print(p)
