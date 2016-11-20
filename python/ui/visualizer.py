import tkinter as tk

import numpy as np

class GeometryVisualizer(tk.Canvas):
    SCALE = 400 # pixels per meter
    FORCE_SIZE = 0.05
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

        self.joints = [
            self.create_arc(-
                -self.FORCE_SIZE,
                -self.FORCE_SIZE,
                self.FORCE_SIZE,
                self.FORCE_SIZE,
                start=0,
                extent=0,
                outline='green',
                width=self.SCALE * 0.0125,
                style='arc'
            )
            for i in range(3)
        ]

        self.__update()

    def on_resize(self, event):
        self.origin = np.array([
             float(event.width), float(event.height)
        ]) * np.array([0.1, 0.5])
        self.__update_once()

    def _to_screen_coords(self, coords):
        return self.origin + self.SCALE * coords

    def __update_once(self):
        last = np.zeros(2)
        poss = self.robot.joint_positions
        angles = self.robot.link_angles
        forces = self.robot.angle_error
        stalled = self.robot.joints_stalled

        for link, pos in zip(self.links, poss):
            coords = np.array([last, pos])
            coords = self._to_screen_coords(coords)
            if np.isnan(coords).any():
                return
            self.coords(link, *coords.ravel())
            last = pos

        for i, joint in enumerate(self.joints):
            posi = poss[i] + self.FORCE_SIZE*np.array([
                [-1, -1],
                [1, 1]
            ])
            coords = self._to_screen_coords(posi)
            self.coords(joint, *coords.ravel())
            extent = np.degrees(forces[i])
            start = -np.degrees(angles[i])

            if extent < 0:
                start += 5
                extent -= 10
            else:
                start -= 5
                extent += 10
            self.itemconfig(joint,
                start=start,
                extent=extent
            )
            if stalled[i]:
                self.itemconfig(joint, outline='red')
            else:
                self.itemconfig(joint, outline='green')

    def __update(self):
        self.__update_once()
        self.after(20, self.__update)
