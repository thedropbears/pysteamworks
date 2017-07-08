from ctre import CANTalon
from wpilib import DoubleSolenoid, Compressor


class Winch:
    # Injectables
    compressor = Compressor
    motor = CANTalon
    rope_lock_solenoid = DoubleSolenoid

    def __init__(self):
        self.locked = False
        self.compressor_enabled = True

    def is_rope_engaged(self):
        """Get whether the rope has been caught by checking the current."""
        return self.motor.getOutputCurrent() > 3

    def is_touchpad_engaged(self):
        """Get whether the touchpad has been engaged by checking the current."""
        return self.motor.getOutputCurrent() > 35

    def rotate(self, value):
        """Rotate winch at specified speed"""
        self.motor.set(value)

    def open_piston(self):
        """Open piston"""
        self.locked = False

    def close_piston(self):
        """Close piston"""
        self.locked = True

    def disable_compressor(self):
        self.compressor_enabled = False

    def enable_compressor(self):
        self.compressor_enabled = True

    def on_enable(self):
        """Run every time the robot transitions to being enabled"""
        self.locked = False
        self.enable_compressor()

    def execute(self):
        """Run at the end of every control loop iteration"""
        if self.locked:
            self.rope_lock_solenoid.set(DoubleSolenoid.Value.kForward)
        else:
            self.rope_lock_solenoid.set(DoubleSolenoid.Value.kReverse)
