import tkinter as tk
import functools

import numpy as np
import weakref

class Bunch:
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)

class Component:
    def __init__(self, owner, id):
        self._owner = weakref.ref(owner)
        self._id = id

    def update(self, coords, **props):
        self._owner().coords(self._id, *coords)
        self._owner().itemconfig(self._id, **props)

    def __repr__(self):
        return "Component({!r}, {!r})".format(self._owner(), self._id)

class BetterCanvas(tk.Canvas):
    def __wrap_create(self, f):
        @functools.wraps(f)
        def wrapped(*args, **kwargs):
            return Component(self, f(*args, **kwargs))
        return wrapped

    def __getattribute__(self, k):
        v = super(BetterCanvas, self).__getattribute__(k)
        if k.startswith('create_'):
            v = self.__wrap_create(v)
        return v

class GeometryVisualizer(BetterCanvas):
    SCALE = 400 # pixels per meter
    FORCE_SIZE = 0.05
    HINGE_RADIUS = 0.0125

    def __init__(self, master, *, robot, **kwargs):
        super().__init__(master, **kwargs)
        self.robot = robot
        self.bind('<Configure>', self.on_resize)

        self.origin = np.zeros(2)

        props = dict(fill='#ff8040', width=self.SCALE * 0.025)

        self.links = [
            self.create_line(0, 0, 0, 0, **props),
            self.create_line(0, 0, 0, 0, **props),
            self.create_line(0, 0, 0, 0, **props),
            self.create_line(0, 0, 0, 0, **props),
        ]

        self.joints = [
            Bunch(
                force_arc=self.create_arc(-
                    -self.FORCE_SIZE,
                    -self.FORCE_SIZE,
                    self.FORCE_SIZE,
                    self.FORCE_SIZE,
                    start=0,
                    extent=0,
                    outline='green',
                    width=self.SCALE * 0.0125,
                    style='arc'
                ),
                hinge=self.create_oval(
                    0, 0, 0, 0,
                    outline='black',
                    fill='white'
                )
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

        # cache the state in case it changes while we're rendering
        s = self.robot.state

        poss = s.joint_positions
        angles = s.link_angles
        forces = s.angle_error
        stalled = s.joints_stalled

        for link, pos in zip(self.links, poss):
            coords = np.array([last, pos])
            coords = self._to_screen_coords(coords)
            if np.isnan(coords).any():
                return
            link.update(coords.ravel())
            last = pos

        for i, joint in enumerate(self.joints):
            pos = poss[i] + self.FORCE_SIZE*np.array([
                [-1, -1],
                [1, 1]
            ])
            coords = self._to_screen_coords(pos)
            extent = -np.degrees(forces[i])
            start = -np.degrees(angles[i+1])

            if extent < 0:
                start += 5
                extent -= 10
            else:
                start -= 5
                extent += 10

            hinge_pos = poss[i] + self.HINGE_RADIUS*np.array([
                [-1, -1],
                [1, 1]
            ])
            hinge_coords = self._to_screen_coords(hinge_pos)
            joint.hinge.update(hinge_coords.ravel())
            joint.force_arc.update(
                coords.ravel(),
                start=start,
                extent=extent,
                outline='red' if stalled[i] else 'green'
            )

    def __update(self):
        self.__update_once()
        self.after(20, self.__update)
