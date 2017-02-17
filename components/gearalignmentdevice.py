from ctre import CANTalon

from networktables import NetworkTable

from components.vision import Vision

class GearAlignmentDevice:

    # this is an injected variable only access after initialization of the
    # robot (so not in __init__)
    gear_alignment_motor = CANTalon
    sd = NetworkTable
    vision = Vision
    l_pos = 0
    r_pos = 10
    zero_pos = (l_pos+r_pos) / 2

    def __init__(self):
        pass

    def setup(self):
        """Run just after createObjects.
        Useful if you want to run something after just once after the
        robot code is started, that depends on injected variables"""

    def on_enable(self):
        """Run every time the robot transitions to being enabled"""
        self.gear_alignment_motor.setControlMode(CANTalon.ControlMode.Position)
        self.gear_alignment_motor.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot)
        self.gear_alignment_motor.setPID(1, 0, 0)

        self.gear_alignment_motor.enableLimitSwitch(True, True)

    def on_disable(self):
        """Run every time the robot transitions to being disabled"""
        pass

    def align(self):
        self.set_postion(self.vision.x)

    def stop_motors(self):
        self.gear_alignment_motor.set(get_rail_pos())

    def left_limit_switch(self):
        return self.gear_alignment_motor.isRevLimitSwitchClosed()

    def right_limit_switch(self):
        return self.gear_alignment_motor.isFwdLimitSwitchClosed()

    def get_rail_pos(self):
        return (self.gear_alignment_motor.getPosition()-self.zero_pos) \
/ (self.r_pos-self.zero_pos)

    def set_postion(self, setpoint):
        self.gear_alignment_motor.set((setpoint+1)*(self.r_pos-self.l_pos))
    
    def reset_postion(self):
        self.gear_alignment_motor.set(self.zero_pos)

    def execute(self):
        """Run at the end of every control loop iteration"""
        #print(self.gear_alignment_motor.getPosition())