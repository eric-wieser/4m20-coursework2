import tkinter as tk
from tkinter import ttk

import config

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
    on_servo_changed = lambda values: None


if __name__ == '__main__':
    from channel import Channel
    import messages

    # Run a simple test of sending a packet, and getting some responses
    with Channel('COM4') as c:
        s = SliderGui(start=[c[0] for c in config.servo_0_90])

        def send_it(v):
            c.write(messages.Control(v))
            print(v)
        s.on_servo_changed = send_it

        def update_it():
            try:
                m = c.read()
            except:
                return

            if isinstance(m, messages.Sensor):
                s.update_ui(m)
        c.on_message = update_it

        tk.mainloop()
