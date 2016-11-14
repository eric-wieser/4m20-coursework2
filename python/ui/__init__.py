from .background import BackgroundTK
from .visualizer import GeometryVisualizer


def basic(r):
    """ a basic user interface that shows the arm orientation """
    import tkinter as tk
    @BackgroundTK
    def gui(root):
        v = GeometryVisualizer(root, robot=r, width=400, height=400)
        v.pack(fill=tk.BOTH)
    return gui
