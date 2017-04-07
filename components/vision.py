import math

from magicbot import tunable
from wpilib import CameraServer, DigitalOutput


class Vision:
    k = tunable(0.5, doc='Weighting of the previous smoothed_x.')

    horizontal_fov = 61 * math.pi/180

    x = tunable(0.0, doc='The centre of the vision target, in the interval [-1.0, 1.0].')
    time = tunable(0, doc='Timestamp of when x was last updated by the vision loop.')
    num_targets = tunable(0, doc='The number of targets visible from the camera.')
    target_sep = tunable(0, doc='If we are seeing the two vision targets - the separation between them as a fraction of image width, else 0.')
    dt = tunable(0)

    enabled = tunable(True, doc='True to enable vision processing.')
    smoothed_x = tunable(0.0, doc='Weighted average of x.')

    led_dio = DigitalOutput

    vision_target_separation = 0.20955 # m

    def __init__(self):
        # CameraServer.launch('vision.py:loop')

        self.led_on = False

    def setup(self):
        """Run just after createObjects.
        Useful if you want to run something after just once after the
        robot code is started, that depends on injected variables"""
        self.led_dio.set(False)

    def on_enable(self):
        """Run every time the robot transitions to being enabled"""
        pass

    def on_disable(self):
        """Run every time the robot transitions to being disabled"""
        self.enabled = True

    def execute(self):
        """Run at the end of the control loop"""
        self.smoothed_x = (1 - self.k) * self.x + self.k * self.smoothed_x

        self.led_dio.set(not (self.enabled or self.led_on))

    def derive_vision_angle(self, vision_x=None):
        "Calculate the camera's angle relative to the vision targets"
        if not vision_x:
            vision_x = self.smoothed_x
        return -(self.horizontal_fov/2 * (vision_x))

    def derive_target_range(self):
        """Estimate the camera's range from the vision targets based of their distance from each other."""
        if abs(self.target_sep) < 0.001:
            return 0.0
        return self.vision_target_separation/(2*math.tan(self.horizontal_fov*self.target_sep/2))

    @classmethod
    def rad_to_vision_units(cls, rad):
        return rad / cls.horizontal_fov
