#!/usr/bin/env python3

import magicbot
import wpilib

from ctre import CANTalon

from components.range_finder import RangeFinder
from components.chassis import Chassis
from components.bno055 import BNO055
from components.gearalignmentdevice import GearAlignmentDevice
from components.geardepositiondevice import GearDepositionDevice
from components.vision import Vision
from components.winch import Winch
from automations.manipulategear import ManipulateGear
from automations.profilefollower import ProfileFollower
from automations.winchautomation import WinchAutomation

from utilities.profilegenerator import generate_trapezoidal_trajectory

from networktables import NetworkTable

import logging

import math


class Robot(magicbot.MagicRobot):

    chassis = Chassis
    gearalignmentdevice = GearAlignmentDevice
    geardepositiondevice = GearDepositionDevice
    winch_automation = WinchAutomation
    manipulategear = ManipulateGear
    vision = Vision
    manipulategear = ManipulateGear
    winch = Winch
    profilefollower = ProfileFollower
    range_finder = RangeFinder

    def createObjects(self):
        '''Create motors and stuff here'''

        # Objects that are created here are shared with all classes
        # that declare them. For example, if I had:
        # self.elevator_motor = wpilib.TalonSRX(2)
        # here, then I could have
        # class Elevator:
        #     elevator_motor = wpilib.TalonSRX
        # and that variable would be available to both the MyRobot
        # class and the Elevator class. This "variable injection"
        # is especially useful if you want to certain objects with
        # multiple different classes.

        # create the imu object
        self.bno055 = BNO055()

        # the "logger" - allows you to print to the logging screen
        # of the control computer
        self.logger = logging.getLogger("robot")
        # the SmartDashboard network table allows you to send
        # information to a html dashboard. useful for data display
        # for drivers, and also for plotting variables over time
        # while debugging
        self.sd = NetworkTable.getTable('SmartDashboard')

        # boilerplate setup for the joystick
        self.joystick = wpilib.Joystick(0)
        self.gamepad = wpilib.Joystick(1)
        self.pressed_buttons_js = set()
        self.pressed_buttons_gp = set()
        self.drive_motor_a = CANTalon(2)
        self.drive_motor_b = CANTalon(5)
        self.drive_motor_c = CANTalon(4)
        self.drive_motor_d = CANTalon(3)
        self.gear_alignment_motor = CANTalon(14)
        self.winch_motor = CANTalon(11)
        self.winch_motor.setInverted(True)
        self.rope_lock_solenoid = wpilib.DoubleSolenoid(forwardChannel=0,
                reverseChannel=1)
        self.gear_push_solenoid = wpilib.Solenoid(2)
        self.gear_drop_solenoid = wpilib.Solenoid(3)

        self.test_trajectory = generate_trapezoidal_trajectory(
                0, 0, 3, 0, Chassis.max_vel, 1, -1)

    def putData(self):
        # update the data on the smart dashboard
        # put the inputs to the dashboard
        self.sd.putNumber("gyro", self.bno055.getHeading())

    def teleopInit(self):
        '''Called when teleop starts; optional'''
        self.sd.putString("state", "stationary")

    def disabledPeriodic(self):
        self.putData()
        self.sd.putString("state", "stationary")

    def teleopPeriodic(self):
        '''Called on each iteration of the control loop'''
        self.putData()
        #self.winch_motor.set(1)
        self.sd.putNumber("climbCurrent", self.winch_motor.getOutputCurrent())

        # if you want to get access to the buttons,
        # you should be doing it like so:
        try:
            if self.debounce(10):
                self.profilefollower.modify_queue(linear=self.test_trajectory)
                self.profilefollower.execute_queue()
        except:
            self.onException()

        try:
            if self.debounce(1, gamepad=True):
                #perform some action

                self.profilefollower.execute_queue()

                self.manipulategear.engage(force=True)

        except:
            self.onException()
        
        try:
            if self.debounce(2, gamepad=True):
                #perform some action
                self.winch_automation.engage(force=True)
        except:
            self.onException()
        
        try:
            if self.debounce(4, gamepad=True):
                #perform some action

                self.profilefollower.execute_queue()

                if self.winch_automation.is_executing:
                    self.winch_automation.done()
                self.winch.rotate_winch(0)

                if self.manipulategear.is_executing:
                    self.manipulategear.done()
                    self.gearalignmentdevice.stop_motors()

                self.sd.putString("state", "stationary")

        except:
            self.onException()
        
        try:
            if self.debounce(3, gamepad=True):
                #perform some action

                self.profilefollower.execute_queue()

                self.winch_automation.done()
                self.winch_motor.set(-0.4)
        except:
            self.onException()

        self.chassis.inputs = [-rescale_js(self.joystick.getY(), deadzone=0.05, exponential=1.2),
                    - rescale_js(self.joystick.getX(), deadzone=0.05, exponential=1.2),
                    - rescale_js(self.joystick.getZ(), deadzone=0.2, exponential=15.0, rate=0.3),
                    (self.joystick.getThrottle() - 1.0) / -2.0
                    ]

    # the 'debounce' function keeps tracks of which buttons have been pressed
    def debounce(self, button, gamepad=False):
        device = None
        if gamepad:
            pressed_buttons = self.pressed_buttons_gp
            device = self.gamepad
        else:
            pressed_buttons = self.pressed_buttons_js
            device = self.joystick
        if device.getRawButton(button):
            if button in pressed_buttons:
                return False
            else:
                pressed_buttons.add(button)
                return True
        else:
            pressed_buttons.discard(button)
            return False

# see comment in teleopPeriodic for information
def rescale_js(value, deadzone=0.0, exponential=0.0, rate=1.0):
    value_negative = 1.0
    if value < 0:
        value_negative = -1.0
        value = -value
    # Cap to be +/-1
    if abs(value) > 1.0:
        value /= abs(value)
    # Apply deadzone
    if abs(value) < deadzone:
        return 0.0
    elif exponential == 0.0:
        value = (value - deadzone) / (1 - deadzone)
    else:
        a = math.log(exponential + 1) / (1 - deadzone)
        value = (math.exp(a * (value - deadzone)) - 1) / exponential
    return value * value_negative * rate

if __name__ == '__main__':
    wpilib.run(Robot)
