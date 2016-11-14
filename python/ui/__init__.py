from .background import BackgroundTK
from .visualizer import GeometryVisualizer


def basic(r):
    """
    A basic user interface that shows the arm orientation

    with Robot.connect() as r, ui.basic(r) as gui:
        while gui.open:
            ...
    """
    import tkinter as tk
    @BackgroundTK
    def gui(root):
        v = GeometryVisualizer(root, robot=r, width=400, height=400)
        v.pack(fill=tk.BOTH)
    return gui
