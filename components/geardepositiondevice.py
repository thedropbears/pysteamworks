import wpilib
from networktables import NetworkTable

class GearDepositionDevice:

    # create solenoids or double solenoids here as we need them, then
    # initialize them in createObjects in robot.py
    sd = NetworkTable
  #  gear_placement_solenoid = DoubleSolenoid

    def __init__(self):
        pass

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

    def placeGear(self):
        pass
       # self.rope_lock_solenoid.set(DoubleSolenoid.Value.kForward)
    
    def reversePiston(self):
        pass
     #   self.rope_lock_solenoid.set(DoubleSolenoid.Value.kReverse)
        
    def execute(self):
        """Run at the end of every control loop iteration"""
        pass
