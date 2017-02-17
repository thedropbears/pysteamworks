import multiprocessing.sharedctypes

import hal

from magicbot import tunable

from vision import vision_loop


class Vision:
    k = tunable(0.5)

    def __init__(self):
        self._data_array = multiprocessing.sharedctypes.RawArray("d", [0.0, 0.0])

        if not hal.isSimulation():
            self._process = multiprocessing.Process(target=vision_loop, args=(self._data_array,), daemon=True)
            self._process.start()

        self.smoothed_x = 0.0

    def setup(self):
        """Run just after createObjects.
        Useful if you want to run something after just once after the
        robot code is started, that depends on injected variables"""
        pass

    def on_enable(self):
        """Run every time the robot transitions to being enabled"""
        pass

    def on_disable(self):
        """Run every time the robot transitions to being disabled"""
        pass

    def execute(self):
        """Run at the end of the control loop"""
        self.smoothed_x = (1 - self.k) * self.x + self.k * self.smoothed_x

    @property
    def x(self):
        return self._data_array[0]

    @property
    def time(self):
        return self._data_array[1]
