from magicbot import tunable
from wpilib import CameraServer


class Vision:
    k = tunable(0.5, doc='Weighting of the previous smoothed_x.')

    x = tunable(0.0, doc='The centre of the vision target, in the interval [-1.0, 1.0].')
    time = tunable(0, doc='Timestamp of when x was last updated by the vision loop.')

    smoothed_x = tunable(0.0, doc='Weighted average of x.')

    def __init__(self):
        CameraServer.launch('vision.py:loop')

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
