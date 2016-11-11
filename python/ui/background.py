import tkinter
import threading

class BackgroundTK:
    """
    A class for running a TK gui in a background thread

    Example:

        import tkinter as tk

        @BackgroundTK
        def gui(root):
            l = tk.Label(root, text='Hello world')
            l.pack()

        with gui:
            # do stuff with the gui in the background

        # gui is closed here

    """
    def __init__(self, func):
        self.open = False

        self._func = func
        self._thread = threading.Thread(target=self.__bg_run)
        self._started = threading.Event()
        self._exc = None
        self._tk = None

    def __bg_run(self):
        """
        Create the TK ui, catching exceptions, and start it

        This should happen in the background thread
        """
        self._tk = tkinter.Tk()
        try:
            self._func(self._tk)
        except Exception as e:
            self._exc = e
            return
        finally:
            self._started.set()

        self.open = True
        self._tk.mainloop()
        self.open = False

        self.__bg_stop();

    def __bg_stop(self):
        # clean up as well as possible
        if self._tk:
            self._tk.quit()
            self._tk = None

    def __enter__(self):
        """
        Start the thread in the background, waiting for the gui to be built
        before proceeding.

        Rethrow any exceptions raised in the background thread whilst
        building the UI
        """
        self._thread.start()
        self._started.wait()
        if self._exc:
            raise self._exc

    def __exit__(self, exc_type, exc_value, exc_tb):
        """
        Close the UI
        """
        if self.open:
            self._tk.after(0, self.__bg_stop)
        self._thread.join()
        self.open = False
        self._tk = None
