from ctre import CANTalon
import math

from networktables import NetworkTable

from components.vision import Vision
from automations.vision_filter import VisionFilter

class GearAlignmentDevice:

    # this is an injected variable only access after initialization of the
    # robot (so not in __init__)
    gear_alignment_motor = CANTalon
    sd = NetworkTable
    vision = Vision
    l_pos = 219 + 50
    r_pos = 689 - 50
    zero_pos = (l_pos+r_pos) / 2

    sp_increment = (r_pos-l_pos)/(0.66*50)

    vision_filter = VisionFilter

    setpoint_leading_timesteps = 5

    def __init__(self):
        self.setpoint = self.zero_pos

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

        self.setpoint = self.gear_alignment_motor.getPosition()

    def on_disable(self):
        """Run every time the robot transitions to being disabled"""
        pass

    def align(self):
        # self.set_position(self.get_rail_pos()+self.vision.x)
        self.set_position(self.get_rail_pos()+self.vision_filter.x+self.vision_filter.dx*self.setpoint_leading_timesteps/50)

    def move_left(self):
        if not self.gear_alignment_motor.getSetpoint()-self.sp_increment < self.l_pos:
            self.setpoint = self.gear_alignment_motor.getSetpoint()-self.sp_increment

    def move_right(self):
        if not self.gear_alignment_motor.getSetpoint()+self.sp_increment > self.r_pos:
            self.setpoint = self.gear_alignment_motor.getSetpoint()+self.sp_increment

    def position_mode(self):
        self.gear_alignment_motor.setControlMode(CANTalon.ControlMode.Position)

    def get_rail_pos(self):
        return (self.gear_alignment_motor.getPosition()-self.zero_pos) \
        / (self.r_pos-self.zero_pos)

    def set_position(self, setpoint):
        self.position_mode()
        sp = ((setpoint+1)/2)*(self.r_pos-self.l_pos)+self.l_pos
        sp = min(self.r_pos, max(self.l_pos, sp))
        self.setpoint = sp

    def reset_position(self):
        self.set_position(0)

    def execute(self):
        """Run at the end of every control loop iteration"""
        self.gear_alignment_motor.set(self.setpoint)