from automations.manipulategear import ManipulateGear
from automations.profilefollower import ProfileFollower
from automations.localisor import Localisor
from components.chassis import Chassis
from components.bno055 import BNO055
from components.vision import Vision
from components.gearalignmentdevice import GearAlignmentDevice
from components.geardepositiondevice import GearDepositionDevice
from components.range_finder import RangeFinder
from magicbot.state_machine import AutonomousStateMachine, state
from utilities.profilegenerator import generate_interpolation_trajectory
from utilities.profilegenerator import generate_trapezoidal_trajectory

import math

class Targets:
    Left = 0
    Center = 1
    Right = 2

class PegAutonomous(AutonomousStateMachine):

    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    chassis = Chassis
    bno055 = BNO055
    vision = Vision
    range_finder = RangeFinder
    gearalignmentdevice = GearAlignmentDevice
    geardepositiondevice = GearDepositionDevice

    center_to_front_bumper = 0.49
    lidar_to_front_bumper = 0.36

    center_airship_distance = 2.93
    side_drive_forward_length = 2.54
    side_rotate_angle = math.pi/3.0
    rotate_accel_speed = 2 # rad*s^-2
    rotate_velocity = 2
    peg_align_tolerance = 0.15
    displace_velocity = Chassis.max_vel
    displace_accel = Chassis.max_acc
    displace_decel = Chassis.max_acc/3
    cut_off_corner = 0.5
    rotate_linear_velocity = 1
    rotate_radius = cut_off_corner/math.tan(math.pi/2 - side_rotate_angle)
    rotate_arc_length = rotate_radius * side_rotate_angle
    rotate_tm = (1/rotate_linear_velocity)*(1/rotate_arc_length)
    rotate_angular_velocity = side_rotate_angle/rotate_tm

    def __init__(self, target=Targets.Center):
        super().__init__()
        self.target = target

    def generate_trajectories(self):
        if self.target is Targets.Left:
            self.perpendicular_heading = -self.side_rotate_angle
            self.forward_displacement = self.side_drive_forward_length-self.center_to_front_bumper-rotate_radius
            self.displace_final_velocity = self.rotate_linear_velocity
        elif self.target is Targets.Right:
            self.perpendicular_heading = self.side_rotate_angle
            self.forward_displacement = self.side_drive_forward_length-self.center_to_front_bumper-rotate_radius
            self.displace_final_velocity = self.rotate_linear_velocity
        else:
            self.perpendicular_heading = 0
            self.forward_displacement = self.center_airship_distance/2-self.center_to_front_bumper
            self.displace_final_velocity = self.displace_velocity

    def on_enable(self):
        super().on_enable()
        self.bno055.resetHeading()
        self.profilefollower.stop()
        self.gearalignmentdevice.reset_position()
        self.geardepositiondevice.retract_gear()
        self.geardepositiondevice.lock_gear()
        self.generate_trajectories()

    @state(first=True)
    def drive_to_airship(self, initial_call):
        # Drive to a range where we can close the loop using vision, lidar and
        # gyro to close the loop on position
        if initial_call:
            self.vision.vision_mode = True
            displace = generate_trapezoidal_trajectory(
                    0, 0, self.dr_displacement,
                    self.rotate_velocity, self.displace_velocity,
                    self.displace_final_velocity, -self.displace_decel)
            self.profilefollower.modify_queue(heading=0, linear=displace)
            self.profilefollower.execute_queue()
        if not self.profilefollower.queue[0]:
            if self.target is Targets.Center:
                self.next_state("drive_to_wall")
            else:
                self.next_state("rotate_towards_airship")


    @state
    def rotate_towards_airship(self, initial_call):
        if initial_call:
            rotate = generate_interpolation_trajectory(0, self.perpendicular_heading, self.rotate_angular_velocity)
            displace = generate_interpolation_trajectory(0, self.rotate_arc_length, self.rotate_linear_velocity)
            self.profilefollower.modify_queue(heading=rotate, overwrite=True)
            print("Rotate Start %s, Rotate End %s" % (rotate[0], rotate[-1]))
            self.profilefollower.execute_queue()
        if not self.profilefollower.queue[0]:
            self.next_state("drive_to_wall")

    @state
    def drive_to_wall(self, initial_call):
        if initial_call:
            to_peg = generate_trapezoidal_trajectory(
                    0, 0, self.range_finder.getDistance()-self.lidar_to_front_bumper,
                    0,
                    self.displace_velocity,
                    self.displace_accel, -self.displace_decel*2)
            self.profilefollower.modify_queue(self.perpendicular_heading,
                    linear=to_peg, overwrite=True)
            self.profilefollower.execute_queue()
            self.manipulategear.engage()
        if not self.manipulategear.is_executing:
            self.next_state("roll_back")

    @state
    def roll_back(self, initial_call):
        if initial_call:
            roll_back_linear = generate_trapezoidal_trajectory(
                    0, 0, -1, 0, 3,
                    self.displace_accel, -self.displace_decel)
            roll_back_angular = generate_interpolation_trajectory(self.perpendicular_heading,
                    0, self.perpendicular_heading/(len(roll_back_linear)/50))
            self.profilefollower.modify_queue(roll_back_angular,
                    linear=roll_back, overwrite=True)
            self.profilefollower.execute_queue()
        if not self.profilefollower.queue[0]:
            self.next_state("drive_around_airship")

    @state
    def drive_around_airship(self, initial_call):
        if initial_call:
            drive_forward_distance
            if self.target == Targets.Left:
                drive_forward_distance = 3
                past_airship_linear = generate_trapezoidal_trajectory(
                        0, 0, drive_forward_distance, 0, 3, self.displace_accel, -self.displace_accel)
                self.profilefollower.modify_queue(heading=0, linear=past_airship_linear)
            elif self.target == Targets.Right:
                drive_around_radius = 5
                around_airship_linear = generate_trapezoidal_trajectory(
                        0, 0, drive_around_radius*math.pi/2, 0, 3, self.displace_accel,
                        -self.displace_accel)
                around_airship_angular = generate_interpolation_trajectory(0, math.pi/2,
                        (math.pi/2/(len(drive_around_linear)/50)))
                self.profilefollower.modify_queue(heading=around_airship_angular, linear=around_airship_linear)
        if not self.profilefollower.queue[0]:
            self.done()

    def done(self):
        super().done()
        self.vision.vision_mode = False


class LeftPegCurves(PegAutonomous):
    MODE_NAME = "Left Peg Curves"

    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    chassis = Chassis

    def __init__(self):
        super().__init__(Targets.Left)


class RightPegCurves(PegAutonomous):
    MODE_NAME = "Right Peg Curves"
    # DEFAULT = True

    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    chassis = Chassis

    def __init__(self):
        super().__init__(Targets.Right)
