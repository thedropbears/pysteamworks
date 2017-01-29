from ctre import CANTalon
from networktables import NetworkTable
from wpilib import DoubleSolenoid

class Winch:

    winch_motor = CANTalon
    rope_lock_solenoid = DoubleSolenoid

    def __init__(self):
        super().__init__()

    def setup(self):
        """Run just after createObjects.
        Useful if you want to run something after just once after the
        robot code is started, that depends on injected variables"""

    def on_enable(self):
        """Run every time the robot transitions to being enabled"""
        pass

    def on_disable(self):
        """Run every time the robot transitions to being disabled"""
        pass

    def execute(self):
        """Run at the end of every control loop iteration"""
        pass
