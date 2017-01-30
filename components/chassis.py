# this is the chassis class, a good example of a component. it is a
# bit special in that it has to take direct numerical input from the
# joystick (which is achieved by changing the 'input' variable, which
# is a list composed of [vx, vy, vz, throttle]) but is a good example
# of what a component should look like.

import wpilib
from ctre import CANTalon

from networktables import NetworkTable

class Chassis:

    drive_motor_a = CANTalon
    drive_motor_b = CANTalon
    drive_motor_c = CANTalon
    drive_motor_d = CANTalon
    sd = NetworkTable

    def __init__(self):
        super().__init__()
        self.inputs = [0.0] * 4

    def setup(self):
        """Run just after createObjects.
        Useful if you want to run something after just once after the
        robot code is started, that depends on injected variables"""
        self.motors = [
                self.drive_motor_a,
                self.drive_motor_b,
                self.drive_motor_c,
                self.drive_motor_d
                ]
        self.motors[1].setControlMode(CANTalon.ControlMode.Follower)
        self.motors[1].set(self.drive_motor_a.getDeviceID())
        self.motors[3].setControlMode(CANTalon.ControlMode.Follower)
        self.motors[3].set(self.drive_motor_c.getDeviceID())
        # reverse two right motors
        self.motors[0].setInverted(False)
        self.motors[1].setInverted(False)
        self.motors[2].setInverted(True)
        self.motors[3].setInverted(True)


    def on_enable(self):
        """Run every time the robot transitions to being enabled"""
        pass

    def on_disable(self):
        """Run every time the robot transitions to being disabled"""
        pass

    def execute(self):
        """Run at the end of every control loop iteration"""
        motor_inputs = [self.inputs[0]-self.inputs[2],
        self.inputs[0]+self.inputs[2]]

        max_i = 1
        for i in motor_inputs:
            if abs(i) > max_i:
                max_i = abs(i)
        for i in motor_inputs:
            i /= max_i
            i *= self.inputs[3]
        self.motors[0].set(motor_inputs[0])
        self.motors[2].set(motor_inputs[1])
