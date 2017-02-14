from ctre import CANTalon

from networktables import NetworkTable

from components.vision import Vision

class GearAlignmentDevice:

    # this is an injected variable only access after initialization of the
    # robot (so not in __init__)
    gear_alignment_motor = CANTalon
    sd = NetworkTable
    vision = Vision

    def __init__(self):
        pass

    def setup(self):
        """Run just after createObjects.
        Useful if you want to run something after just once after the
        robot code is started, that depends on injected variables"""

    def on_enable(self):
        """Run every time the robot transitions to being enabled"""
        #self.gear_alignment_motor.setControlMode(CANTalon.ControlMode.Position)
        #self.gear_alignment_motor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder)
    def on_disable(self):
        """Run every time the robot transitions to being disabled"""
        pass

    def align(self, value):
        self.gear_alignment_motor.set(value)

    def stop_motors(self):
        self.gear_alignment_motor.set(0)

    def execute(self):
        """Run at the end of every control loop iteration"""