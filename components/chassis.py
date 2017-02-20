# this is the chassis class, a good example of a component. it is a
# bit special in that it has to take direct numerical input from the
# joystick (which is achieved by changing the 'input' variable, which
# is a list composed of [vx, vy, vz, throttle]) but is a good example
# of what a component should look like.

import wpilib
from ctre import CANTalon

import math

from networktables import NetworkTable

class Chassis:

    drive_motor_a = CANTalon
    drive_motor_b = CANTalon
    drive_motor_c = CANTalon
    drive_motor_d = CANTalon
    sd = NetworkTable

    inches_to_meters = 0.0254
    # some calculations that provide numbers used by the motion profiling
    wheel_circumference = 6*inches_to_meters*math.pi # m
    counts_per_revolution = 1024
    # convert from sensor units to meters
    counts_per_meter = counts_per_revolution/wheel_circumference

    # to convert from m/s to counts per 100ms
    velocity_to_native_units = 0.1*counts_per_meter

    # max velocity as measured by talons driving flat out
    max_vel_native = 800 # ticks / 100ms
    # convert to SI units - m/s
    max_vel = (10*max_vel_native*wheel_circumference)/counts_per_revolution
    max_acc = 2 # m/s

    wheelbase_width = 0.629666 # m

    pid_profile = {
            "kP": 1,
            "kI": 0.01,
            "kD": 10,
            "kF": 1023//max_vel_native,
            "ramp-rate" : 36 # change in volts, in v/sec
    }

    motion_profile_speed = 50 # Hz


    def __init__(self):
        super().__init__()
        self.inputs = [0.0] * 4

        self.input_enabled = True

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

        self.motors[0].setPID(
                self.pid_profile["kP"],
                self.pid_profile["kI"],
                self.pid_profile["kD"],
                f = self.pid_profile["kF"],
                )

        self.motors[2].setPID(
                self.pid_profile["kP"],
                self.pid_profile["kI"],
                self.pid_profile["kD"],
                f = self.pid_profile["kF"],
                )

        self.motors[0].setProfile(0)
        self.motors[2].setProfile(0)

        self.motors[0].setVoltageRampRate(self.pid_profile["ramp-rate"])
        self.motors[2].setVoltageRampRate(self.pid_profile["ramp-rate"])

        self.motors[0].setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder)
        self.motors[2].setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder)

        self.motors[0].setControlMode(CANTalon.ControlMode.Speed)
        self.motors[2].setControlMode(CANTalon.ControlMode.Speed)

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
        """Run by magicbot when the robot is enabled."""
        self.input_enabled = True

    def enable_input(self):
        self.input_enabled = True

    def disable_input(self):
        self.input_enabled = False

    def set_enc_pos(self, left=0, right=0):
        self.motors[0].setPosition(int(left))
        self.motors[2].setPosition(int(right))

    def get_wheel_distances(self):
        return [self.motors[0].getPosition()/self.counts_per_meter,
                -self.motors[2].getPosition()/self.counts_per_meter]

    def set_velocity(self, linear, angular):
        angular *= Chassis.wheelbase_width/2
        left_out = linear - angular
        right_out = linear + angular

        self.motors[0].set(left_out*Chassis.velocity_to_native_units)
        self.motors[2].set(right_out*Chassis.velocity_to_native_units)

    def execute(self):
        """Run at the end of every control loop iteration"""
        if self.input_enabled:
            motor_inputs = [self.inputs[0]-self.inputs[2],
                    self.inputs[0]+self.inputs[2]]

            max_i = 1
            for i in motor_inputs:
                if abs(i) > max_i:
                    max_i = abs(i)
            for i in range(len(motor_inputs)):
                motor_inputs[i] /= max_i
                motor_inputs[i] *= self.inputs[3]
            self.motors[0].set(motor_inputs[0]*Chassis.max_vel_native)
            self.motors[2].set(motor_inputs[1]*Chassis.max_vel_native)
