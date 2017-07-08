#!/usr/bin/env python3

import math
import time

import magicbot
import wpilib

from ctre import CANTalon
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from networktables import NetworkTable

from components.bno055 import BNO055
from components.chassis import Chassis
from components.gears import GearAlignmentDevice, GearDepositionDevice
from components.range_finder import RangeFinder
from components.vision import Vision
from components.winch import Winch
from automations.filters import RangeFilter, VisionFilter
from automations.manipulategear import ManipulateGear
from automations.profilefollower import ProfileFollower
from automations.winch import WinchAutomation


class Robot(magicbot.MagicRobot):
    # Sensors
    range_finder = RangeFinder
    vision = Vision
    vision_filter = VisionFilter

    # Chassis must come before RangeFilter
    # ProfileFollower should come before Chassis
    profilefollower = ProfileFollower
    chassis = Chassis
    range_filter = RangeFilter

    # Other automations
    manipulategear = ManipulateGear
    winch_automation = WinchAutomation

    # Other actuators
    gearalignmentdevice = GearAlignmentDevice
    geardepositiondevice = GearDepositionDevice
    winch = Winch

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

        # the SmartDashboard network table allows you to send
        # information to a html dashboard. useful for data display
        # for drivers, and also for plotting variables over time
        # while debugging
        self.sd = NetworkTable.getTable('SmartDashboard')

        # create the joystick and gamepad on the appropriate ports
        self.joystick = wpilib.Joystick(0)
        self.gamepad = wpilib.XboxController(1)

        # create the button debouncers, used to stop a single press from being
        # counted every control loop iteration
        self.joystick_buttons = [ButtonDebouncer(
                self.joystick, buttonnum=i, period=0.5) for i in range(13)]
        self.gamepad_buttons = [ButtonDebouncer(
                self.gamepad, buttonnum=i, period=0.5) for i in range(13)]

        self.drive_motor_a = CANTalon(2)
        self.drive_motor_b = CANTalon(5)
        self.drive_motor_c = CANTalon(4)
        self.drive_motor_d = CANTalon(3)
        self.gear_alignment_motor = CANTalon(14)

        # create the winch motor; set it so that it pulls the robot up
        # the rope for a positive setpoint
        self.winch_motor = CANTalon(11)
        self.winch_motor.setInverted(True)

        self.rope_lock_solenoid = wpilib.DoubleSolenoid(forwardChannel=0,
                reverseChannel=1)

        self.gear_push_solenoid = wpilib.Solenoid(2)
        self.gear_drop_solenoid = wpilib.Solenoid(3)
        self.gear_drop_solenoid.set(True)

        # set the throttle to be sent to the chassis
        self.throttle = 1.0
        # set the direction of the forward/back movement of the robot
        self.direction = 1.0

        self.range_finder_counter = wpilib.Counter(0, mode=wpilib.Counter.Mode.kPulseLength)
        # dio for the vision LEDs to be switched on or off
        self.led_dio = wpilib.DigitalOutput(1)

        self.compressor = wpilib.Compressor()

    def putData(self):
        # update the data on the smart dashboard

        # put the inputs to the dashboard
        self.sd.putNumber("gyro", self.bno055.getHeading())
        # if self.manipulategear.current_state == "align_peg":
        self.sd.putNumber("range", self.range_finder.getDistance())
        self.sd.putNumber("climb_current", self.winch_motor.getOutputCurrent())
        self.sd.putNumber("rail_pos", self.gearalignmentdevice.get_rail_pos())
        self.sd.putNumber("raw_rail_pos", self.gear_alignment_motor.getPosition())
        self.sd.putNumber("error_differential",
            self.drive_motor_a.getClosedLoopError()
            - self.drive_motor_c.getClosedLoopError())
        self.sd.putNumber("velocity", self.chassis.get_velocity())
        self.sd.putNumber("left_speed_error", self.drive_motor_a.getClosedLoopError())
        self.sd.putNumber("right_speed_error", self.drive_motor_c.getClosedLoopError())
        self.sd.putNumber("x_throttle", self.chassis.inputs[0])
        self.sd.putNumber("z_throttle", self.chassis.inputs[2])
        self.sd.putNumber("filtered_x", self.vision_filter.x)
        self.sd.putNumber("filtered_dx", self.vision_filter.dx)
        self.sd.putNumber("vision_filter_x_variance", self.vision_filter.filter.P[0, 0])
        self.sd.putNumber("vision_filter_dx_variance", self.vision_filter.filter.P[1, 1])
        self.sd.putNumber("vision_filter_covariance", self.vision_filter.filter.P[0, 1])
        self.sd.putNumber("filtered_range", self.range_filter.filter.x_hat[0, 0])
        self.sd.putNumber("range_filter_variance", self.range_filter.filter.P[0, 0])
        self.sd.putNumber("time", time.time())
        self.sd.putNumber("vision_predicted_range", self.range_filter.vision_predicted_range())
        self.sd.putNumber("vision_predicted_target_dist", self.vision.derive_target_range())

    def teleopInit(self):
        '''Called when teleop starts; optional'''
        self.sd.putString("state", "stationary")
        self.gearalignmentdevice.reset_position()
        self.geardepositiondevice.retract_gear()
        self.geardepositiondevice.lock_gear()
        self.profilefollower.stop()
        self.winch.enable_compressor()
        self.vision.enabled = False
        self.logger.info("TELEOP INIT RANGE: %s" % (self.range_finder.getDistance()))
        self.logger.info("TELEOP INIT FILTER RANGE: %s" % (self.range_filter.range))

    def disabledInit(self):
        # prevent pyntlogger from continuing to log if it is running
        self.sd.putBoolean("log", False)

    def disabledPeriodic(self):
        self.putData()
        self.vision_filter.execute()
        self.range_filter.execute()

    def teleopPeriodic(self):
        '''Called on each iteration of the control loop'''
        self.putData()

        # enable the compressor only if the chassis is moving below the
        # threshold speed, and the winch is not running, in order to prevent
        # brownouts
        self.compressor.setClosedLoopControl(self.chassis.compressor_enabled
                and self.winch.compressor_enabled)

        # check for button inputs

        # force restart or start of the gear state machine
        with self.consumeExceptions():
            if self.gamepad_buttons[8].get() or self.joystick_buttons[1].get():
                self.manipulategear.engage(force=True)

        # force restart or start of the winch state machine
        with self.consumeExceptions():
            if self.gamepad_buttons[7].get() or self.joystick_buttons[3].get():
                self.winch_automation.engage(force=True)

        # tell pyntlogger that we want to start logging
        with self.consumeExceptions():
            if self.joystick_buttons[7].get():
                self.sd.putBoolean("log", True)

        # reset the gear & winch state machines
        with self.consumeExceptions():
            if self.joystick_buttons[2].get():
                if self.manipulategear.is_executing:
                    self.manipulategear.done()
                self.gearalignmentdevice.reset_position()
                self.geardepositiondevice.retract_gear()
                self.geardepositiondevice.lock_gear()

        # force stop the winch state machine and winch motor
        with self.consumeExceptions():
            if self.joystick_buttons[4].get():
                if self.winch_automation.is_executing:
                    self.winch_automation.done()
                self.winch.rotate_winch(0)

        # force the winch motor to spin at max speed, and close the piston
        # that holds the rope in place
        with self.consumeExceptions():
            if self.joystick_buttons[5].get():
                if self.winch_automation.is_executing:
                    self.winch_automation.done()
                self.winch.rotate_winch(1.0)
                self.winch.piston_close()

        # toggle the position of the winch piston
        with self.consumeExceptions():
            if self.joystick_buttons[6].get():
                self.winch.locked = not self.winch.locked

        # retract and lock the gear bucket
        with self.consumeExceptions():
            if self.joystick_buttons[12].get():
                self.geardepositiondevice.retract_gear()
                self.geardepositiondevice.lock_gear()

        # push the gear bucket forward while shutting it
        with self.consumeExceptions():
            if self.joystick_buttons[10].get() or self.gamepad_buttons[1].get():
                self.manipulategear.engage(initial_state="forward_closed", force=True)

        # Set direction and speed of control inputs from driver when specific
        # buttons are pressed
        if (not self.gamepad.getRawButton(5) and
                not self.gamepad.getRawButton(6) and
                not self.gamepad.getRawAxis(3) > 0.9):
            # normal operating mode
            self.throttle = 1
            self.direction = 1
            self.sd.putString("camera", "front")
        elif self.gamepad.getRawButton(5):
            # reverse direction of translation
            self.throttle = 1
            self.direction = -1
            self.sd.putString("camera", "back")
        elif self.gamepad.getRawButton(6):
            # slow down
            self.throttle = 0.5
            self.direction = 1
            self.sd.putString("camera", "back")
        elif self.gamepad.getRawAxis(3) > 0.9:
            # slow down and reverse direction of translation
            self.throttle = 0.5
            self.direction = -1
            self.sd.putString("camera", "back")

        # POV buttons on joystick move gear rail left and right for alignment
        # with chute
        if self.joystick.getPOV() == 90:
            if not self.manipulategear.is_executing:
                self.gearalignmentdevice.move_right()
        elif self.joystick.getPOV() == 270:
            if not self.manipulategear.is_executing:
                self.gearalignmentdevice.move_left()

        if 1.5 < abs(self.chassis.get_velocity()) and not self.manipulategear.is_executing:
            self.gearalignmentdevice.set_position(0)

        # set control inputs to chassis after rescaling
        linear_input = (
            self.direction * -rescale_js(
                    self.gamepad.getRawAxis(1), deadzone=0.05, exponential=30))
        # rotational speed. compensate for reduced throttle by increasing
        # rate, so that when linear speed is reduced, rotational input stays
        # the same for the drivers
        angular_input = -rescale_js(
                self.gamepad.getRawAxis(4), deadzone=0.05, exponential=30,
                rate=0.3 if self.throttle == 1 else 1/self.throttle)
        # y axis (left right) input is 0, as a tank drive can not translate
        # sideways
        self.chassis.inputs = [linear_input, 0, angular_input, self.throttle]

        # allow co-driver to manually turn on the vision LEDs
        self.vision.led_on = self.joystick.getRawButton(11)


# utility function to rescale joystick inputs and make them exponential rather
# than linear
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
