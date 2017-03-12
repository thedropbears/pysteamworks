from ctre import CANTalon
import math

from networktables import NetworkTable

from components.vision import Vision

class GearAlignmentDevice:

    # this is an injected variable only access after initialization of the
    # robot (so not in __init__)
    gear_alignment_motor = CANTalon
    sd = NetworkTable
    vision = Vision
    l_pos = 257 + 50
    r_pos = 641 - 50
    zero_pos = (l_pos+r_pos) / 2

    sp_increment = (r_pos-l_pos)/(0.66*50)

    def __init__(self):
        pass

    def setup(self):
        """Run just after createObjects.
        Useful if you want to run something after just once after the
        robot code is started, that depends on injected variables"""

    def on_enable(self):
        """Run every time the robot transitions to being enabled"""
        self.gear_alignment_motor.clearStickyFaults()

        self.gear_alignment_motor.setControlMode(CANTalon.ControlMode.Position)
        self.gear_alignment_motor.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot)

        self.gear_alignment_motor.setPID(20, 0, 0)

        self.gear_alignment_motor.enableLimitSwitch(True, True)

    def on_disable(self):
        """Run every time the robot transitions to being disabled"""
        pass

    def align(self):
        self.set_position(self.get_rail_pos()+self.vision.x)

    def move_left(self):
        if not self.gear_alignment_motor.getSetpoint()-self.sp_increment < self.l_pos:
            self.gear_alignment_motor.set(self.gear_alignment_motor.getSetpoint()-self.sp_increment)

    def move_right(self):
        if not self.gear_alignment_motor.getSetpoint()+self.sp_increment > self.r_pos:
            self.gear_alignment_motor.set(self.gear_alignment_motor.getSetpoint()+self.sp_increment)
        print("Move right, sp %s, pos increment %s" % (self.gear_alignment_motor.getSetpoint(), self.sp_increment))

    def position_mode(self):
        self.gear_alignment_motor.setControlMode(CANTalon.ControlMode.Position)

    def stop_motors(self):
        self.gear_alignment_motor.stopMotor()

    def get_rail_pos(self):
        return (self.gear_alignment_motor.getPosition()-self.zero_pos) \
        / (self.r_pos-self.zero_pos)

    def set_position(self, setpoint):
        self.position_mode()
        sp = ((setpoint+1)/2)*(self.r_pos-self.l_pos)+self.l_pos
        sp = min(self.r_pos, max(self.l_pos, sp))
        self.gear_alignment_motor.set(sp)

    def reset_position(self):
        self.set_position(0)

    def execute(self):
        """Run at the end of every control loop iteration"""