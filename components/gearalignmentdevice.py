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
    l_pos = 255
    r_pos = 670
    zero_pos = (l_pos+r_pos) / 2 #462.5

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
        # if (abs(self.gear_alignment_motor.getClosedLoopError()) <
        #         0.03*(self.r_pos-self.l_pos)/2):
        self.set_position(self.get_rail_pos()+self.vision.x)

    def move_left(self):
        if not self.left_limit_switch():
            self.gear_alignment_motor.setControlMode(CANTalon.ControlMode.PercentVbus)
            self.gear_alignment_motor.set(-1)

    def move_right(self):
        if not self.right_limit_switch():
            self.gear_alignment_motor.setControlMode(CANTalon.ControlMode.PercentVbus)
            self.gear_alignment_motor.set(1)

    def position_mode(self):
        self.gear_alignment_motor.setControlMode(CANTalon.ControlMode.Position)

    def stop_motors(self):
        self.gear_alignment_motor.stopMotor()

    def left_limit_switch(self):
        return self.gear_alignment_motor.isRevLimitSwitchClosed()

    def right_limit_switch(self):
        return self.gear_alignment_motor.isFwdLimitSwitchClosed()

    def get_rail_pos(self):
        return (self.gear_alignment_motor.getPosition()-self.zero_pos) \
        / (self.r_pos-self.zero_pos)

    def set_position(self, setpoint):
        sp = ((setpoint+1)/2)*(self.r_pos-self.l_pos)+self.l_pos
        sp = min(self.r_pos, max(self.l_pos, sp))
        self.gear_alignment_motor.set(sp)

    def reset_position(self):
        self.set_position(0)

    def execute(self):
        """Run at the end of every control loop iteration"""