import wpilib
from networktables import NetworkTable

class GearDepositionDevice:

    # create solenoids or double solenoids here as we need them, then
    # initialize them in createObjects in robot.py
    sd = NetworkTable
    gear_push_solenoid = wpilib.Solenoid
    gear_drop_solenoid = wpilib.Solenoid

    def __init__(self):
        self.push_piston = False
        self.drop_piston = False

    def setup(self):
        """Run just after createObjects.
        Useful if you want to run something after just once after the
        robot code is started, that depends on injected variables"""

    def on_enable(self):
        """Run every time the robot transitions to being enabled"""

    def on_disable(self):
        """Run every time the robot transitions to being disabled"""
        pass

    def push_gear(self):
        self.push_piston = True

    def retract_gear(self):
        self.push_piston = False

    def drop_gear(self):
        self.drop_piston = True

    def lock_gear(self):
        self.drop_piston = False

    def execute(self):
        """Run at the end of every control loop iteration"""
        self.gear_push_solenoid.set(self.push_piston)
        self.gear_drop_solenoid.set(self.drop_piston)
