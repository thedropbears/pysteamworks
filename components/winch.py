from ctre import CANTalon
from wpilib import DoubleSolenoid, Compressor

from collections import deque
import numpy as np

class Winch:

    winch_motor = CANTalon
    rope_lock_solenoid = DoubleSolenoid
    compressor = Compressor

    def __init__(self):
        super().__init__()

        self.locked = False
        self.compressor_enabled = True

    def on_rope_engaged(self):
        """Return wether the current is over 5 as a boolean"""
        return self.winch_motor.getOutputCurrent() > 10

    def on_touchpad_engaged(self):
        """Return wether the current is over 2 as a boolean"""
        return self.winch_motor.getOutputCurrent() > 35

    def rotate_winch(self, value):
        """Rotate winch motor with half speed"""
        self.winch_motor.set(value)

    def piston_open(self):
        """Open piston"""
        self.locked = False

    def piston_close(self):
        """Close piston"""
        self.locked = True

    def disable_compressor(self):
        self.compressor_enabled = False

    def enable_compressor(self):
        self.compressor_enabled = True

    def setup(self):
        """Run just after createObjects.
        Useful if you want to run something after just once after the
        robot code is started, that depends on injected variables"""

    def on_enable(self):
        """Run every time the robot transitions to being enabled"""
        self.locked = False
        self.enable_compressor()

    def on_disable(self):
        """Run every time the robot transitions to being disabled"""
        pass

    def execute(self):
        """Run at the end of every control loop iteration"""
        if self.locked:
            self.rope_lock_solenoid.set(DoubleSolenoid.Value.kForward)
        else:
            self.rope_lock_solenoid.set(DoubleSolenoid.Value.kReverse)

        self.compressor.setClosedLoopControl(self.compressor_enabled)

