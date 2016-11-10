import tkinter as tk
from tkinter import ttk

import config
import time

class SliderGui:
    """
    A GUI that shows a set of sliders to control the motors and read the sensors
    """
    def __init__(self, start=[1500] * config.N):
        # create the window
        root = tk.Tk()
        root.resizable(True, False)
        root.minsize(800, 0)

        self._pot_sliders = []
        self._root = root
        self._servo_values = start

        # create the servo sliders
        servo_sliders = ttk.LabelFrame(root, text="Servo pulse widths")
        servo_sliders.pack(padx=10, pady=10, side=tk.LEFT, fill=tk.BOTH, expand=True)
        for i in range(config.N):
            scale = tk.Scale(servo_sliders,
                from_=config.servo_limits[0], to=config.servo_limits[1],
                tickinterval=250, orient=tk.HORIZONTAL, takefocus=1,
                command=lambda evt, i=i: self._servo_changed(evt, i))
            scale.pack(fill=tk.BOTH)
            scale.set(self._servo_values[i])

        # create the potentiometer sliders
        pot_sliders = ttk.LabelFrame(root, text="Analog sensor readings")
        pot_sliders.pack(padx=10, pady=10, side=tk.LEFT, fill=tk.BOTH, expand=True)
        for i in range(config.N):
            scale = tk.Scale(pot_sliders, from_=0, to=1024, tickinterval=128,
                # state=tk.DISABLED,
                orient=tk.HORIZONTAL)
            scale.pack(fill=tk.BOTH)
            self._pot_sliders.append(scale)

        self._on_servo_changed = lambda values: None

    def _servo_changed(self, val, i):
        """ called when a single slider value changes"""
        self._servo_values[i] = int(val)
        self.on_servo_changed(self._servo_values)

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
        callback(self._servo_values)


if __name__ == '__main__':
    from channel import Channel, NoMessage
    import messages

    # Run a simple test of sending a packet, and getting some responses
    with Channel('COM4') as c:
        time.sleep(0.25)
        gui = SliderGui(start=[c[0] for c in config.servo_0_90])

        def send_it(v):
            c.write(messages.ServoPulse(v))
        gui.on_servo_changed = send_it

        def update_it():
            sensor_m = None
            while True:
                try:
                    m = c.read(timeout=0)
                except NoMessage:
                    break
                except Exception as e:
                    print(e)
                    continue

                if isinstance(m, messages.Sensor):
                    sensor_m = m
                elif isinstance(m, messages.IMUScaled):
                    pass

            if sensor_m:
                gui.update_ui(sensor_m)

            gui._root.after(10, update_it)

        gui._root.after(0, update_it)

        gui._root.mainloop()

        c.write(messages.ServoPulse((0xFFFF,)*config.N))
