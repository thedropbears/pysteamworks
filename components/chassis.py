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
    counts_per_revolution = 1440
    # convert from sensor units to meters
    counts_per_meter = counts_per_revolution/wheel_circumference

    # to convert from m/s to counts per 100ms
    velocity_to_native_units = 0.1*counts_per_meter

    # max velocity as measured by talons driving flat out
    max_vel_native = 1100 # ticks / 100ms
    # convert to SI units - m/s
    max_vel = (10*max_vel_native*wheel_circumference)/counts_per_revolution
    max_acc = 3 # m/s

    wheelbase_width = 0.629666 # m

    pid_profile = {
            "kP": 1,
            "kI": 0,
            "kD": 3,
            "kF": 1*(1023/max_vel_native),
            "ramp-rate" : 72 # change in volts, in v/sec
    }

    motion_profile_freq = 50 # Hz

    compressor = wpilib.Compressor


    def __init__(self):
        super().__init__()
        self.inputs = [0.0] * 4

        self.input_enabled = True
        self.mp_enabled = False
        self.compressor_enabled = True

    def setup(self):
        """Setup the motors"""

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

        self.set_enc_pos()


    def on_enable(self):
        """Run by magicbot when the robot is enabled."""
        self.input_enabled = True

    def enable_input(self):
        """Enable operator control of chassis"""
        self.input_enabled = True

    def disable_input(self):
        """Disable operator control of chassis"""
        self.input_enabled = False

    def set_enc_pos(self, left=0, right=0):
        """Reset the encoder positions to a certain value"""
        self.offset_positions = [self.motors[0].getPosition()/self.counts_per_meter+left,
                self.motors[2].getPosition()/self.counts_per_meter-right]

    def get_wheel_distances(self):
        """Return the distances that the wheels have travelled, minus the offset"""
        return [self.motors[0].getPosition()/self.counts_per_meter-self.offset_positions[0],
                -(self.motors[2].getPosition()/self.counts_per_meter-self.offset_positions[1])]

    def get_raw_wheel_distances(self):
        """Return the raw distances that the wheels have travelled"""
        return [self.motors[0].getPosition()/self.counts_per_meter,
                -self.motors[2].getPosition()/self.counts_per_meter]

    def get_velocities(self):
        """Return the velocity of the left and right sides of the robot in
        SI units."""
        return [self.motors[0].getEncVelocity()/self.velocity_to_native_units,
                -self.motors[2].getEncVelocity()/self.velocity_to_native_units]

    def get_velocity(self):
        """Return the average velocity of the left and right sides of robot"""
        return (self.get_velocities()[0]+self.get_velocities()[1])/2

    def set_velocity(self, linear, angular):
        """ Function to allow the motion profiling code to set the speed
        setpoints of the chassis.
        :param linear: linear speed setpoint for the robot. m/s
        :param angular: angular velocity setpoint for the robot. rad/s
        """
        self.mp_enabled = True
        angular *= Chassis.wheelbase_width/2
        left_out = linear - angular
        right_out = linear + angular

        self.mp_setpoints = [
        1023/self.counts_per_revolution*left_out*Chassis.velocity_to_native_units,
        1023/self.counts_per_revolution*right_out*Chassis.velocity_to_native_units]

    def execute(self):
        """Run at the end of every control loop iteration"""
        if self.mp_enabled:
            self.mp_enabled = False
            self.motors[0].set(self.mp_setpoints[0])
            self.motors[2].set(self.mp_setpoints[1])
        elif self.input_enabled:
            motor_inputs = [self.inputs[0]-self.inputs[2],
                    self.inputs[0]+self.inputs[2]]

            max_i = 1
            for i in motor_inputs:
                if abs(i) > max_i:
                    max_i = abs(i)
            for i in range(len(motor_inputs)):
                motor_inputs[i] /= max_i
                motor_inputs[i] *= self.inputs[3]
            # disable compressor if inputs above certain level -
            # prevent brownouts
            if abs(self.inputs[0]) > 0.5 or abs(self.inputs[2]) > 0.5:
                self.compressor_enabled = False
            else:
                self.compressor_enabled = True
            self.motors[0].set(motor_inputs[0]*Chassis.max_vel_native)
            self.motors[2].set(motor_inputs[1]*Chassis.max_vel_native)
            if motor_inputs[0] == 0 and motor_inputs[1] == 0:
                self.motors[0].clearIaccum()
                self.motors[1].clearIaccum()
