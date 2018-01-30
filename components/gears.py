import wpilib
from ctre import CANTalon
from networktables import NetworkTable

from automations.filters import VisionFilter


class GearAligner:
    # this is an injected variable only access after initialization of the
    # robot (so not in __init__)
    motor = CANTalon
    sd = NetworkTable
    vision_filter = VisionFilter

    r_pos = -230 - 50
    l_pos = -730 + 50
    zero_pos = (l_pos+r_pos) / 2

    sp_increment = (r_pos-l_pos)/(0.66*50)

    setpoint_leading_timesteps = 10

    def __init__(self):
        self.setpoint = self.zero_pos

    def on_enable(self):
        """Run every time the robot transitions to being enabled"""
        self.motor.clearStickyFaults()

        self.motor.setControlMode(CANTalon.ControlMode.Position)
        self.motor.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot)

        self.motor.setPID(20, 0, 0)

        # self.motor.enableLimitSwitch(True, True)
        self.motor.enableLimitSwitch(False, False)

        self.setpoint = self.motor.getPosition()

        self.motor.reverseSensor(True)

    def align(self):
        self.set_position(self.get_rail_pos() + self.vision_filter.x
                          + self.vision_filter.dx*self.setpoint_leading_timesteps/50)

    def move_left(self):
        if not self.motor.getSetpoint()-self.sp_increment < self.l_pos:
            self.setpoint = self.motor.getSetpoint()-self.sp_increment

    def move_right(self):
        if not self.motor.getSetpoint()+self.sp_increment > self.r_pos:
            self.setpoint = self.motor.getSetpoint()+self.sp_increment

    def get_rail_pos(self):
        return ((self.motor.getPosition()-self.zero_pos)
                / (self.r_pos-self.zero_pos))

    def set_position(self, setpoint):
        sp = ((setpoint+1)/2)*(self.r_pos-self.l_pos)+self.l_pos
        sp = min(self.r_pos, max(self.l_pos, sp))
        self.setpoint = sp

    def reset_position(self):
        self.set_position(0)

    def execute(self):
        """Run at the end of every control loop iteration"""
        self.motor.set(self.setpoint)


class GearDepositor:
    # create solenoids or double solenoids here as we need them, then
    # initialize them in createObjects in robot.py
    gear_drop_solenoid = wpilib.Solenoid
    gear_push_solenoid = wpilib.Solenoid
    sd = NetworkTable

    def __init__(self):
        self.push_piston = False
        self.drop_piston = True

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
        self.gear_drop_solenoid.set(not self.drop_piston)
