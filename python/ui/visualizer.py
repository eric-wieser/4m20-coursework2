import tkinter as tk

import numpy as np

class GeometryVisualizer(tk.Canvas):
    SCALE = 400 # pixels per meter
    def __init__(self, master, *, robot, **kwargs):
        super().__init__(master, **kwargs)
        self.robot = robot
        self.bind('<Configure>', self.on_resize)

        self.origin = np.zeros(2)

        props = dict(fill='red', width=self.SCALE * 0.025)

        self.links = [
            self.create_line(0, 10, 20, 30, **props),
            self.create_line(0, 0, 200, 0, **props),
            self.create_line(0, 0, 200, 0, **props),
            self.create_line(0, 0, 200, 0, **props),
        ]

        self.__update()

    def on_resize(self, event):
        self.origin = np.array([
             float(event.width), float(event.height)
        ]) / 2
        self.__update_once()

    def _to_screen_coords(self, coords):
        return self.origin + self.SCALE * coords

    def __update_once(self):
        last = np.zeros(2)
        for link, pos in zip(self.links, self.robot.joint_positions):
            coords = np.array([last, pos])
            coords = self._to_screen_coords(coords)
            self.coords(link, *coords.ravel())
            last = pos

    def __update(self):
        self.__update_once()
        self.after(20, self.__update)
