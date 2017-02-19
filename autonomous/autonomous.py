from automations.manipulategear import ManipulateGear
from automations.profilefollower import ProfileFollower
from components.chassis import Chassis
from components.bno055 import BNO055
from components.vision import Vision
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

    center_to_front_bumper = 0.49
    lidar_to_front_bumper = 0.36

    center_tower_distance = 3
    side_drive_forward_length = 1.5
    side_rotate_angle = math.pi/3.0
    # rotate_accel_speed = 0.25 # rad*s^-2
    # rotate_velocity = 1
    rotate_accel_speed = 2 # rad*s^-2
    rotate_velocity = 2

    def __init__(self, target=Targets.Center):
        super().__init__()
        self.target = target

        if target == Targets.Center:
            self.displace_0 = generate_trapezoidal_trajectory(
                    0, 0, self.center_tower_distance, 0, Chassis.max_vel/3,
                    Chassis.max_acc, -Chassis.max_acc)
            self.rotate_0 = []
        elif target == Targets.Left:
            self.displace_0 = generate_trapezoidal_trajectory(
                    0, 0, self.side_drive_forward_length, 0,
                    Chassis.max_vel/3,
                    Chassis.max_acc, -Chassis.max_acc)
            self.rotate_0 = generate_trapezoidal_trajectory(
                    0, 0, -self.side_rotate_angle, 0, self.rotate_velocity,
                    self.rotate_accel_speed, -self.rotate_accel_speed)
        else:
            self.displace_0 = generate_trapezoidal_trajectory(
                    0, 0, self.side_drive_forward_length, 0,
                    Chassis.max_vel/3,
                    Chassis.max_acc, -Chassis.max_acc)
            self.rotate_0 = generate_trapezoidal_trajectory(
                    0, 0, self.side_rotate_angle, 0, self.rotate_velocity,
                    self.rotate_accel_speed, -self.rotate_accel_speed)

    def on_enable(self):
        super().on_enable()
        self.bno055.resetHeading()
        self.profilefollower.stop()

    @state(first=True)
    def drive_to_airship(self, initial_call):
        # Drive to a range where we can close the loop using vision, lidar and
        # gyro to close the loop on position
        if initial_call:
            self.profilefollower.modify_queue(heading=0,
                    linear=self.displace_0)

            self.profilefollower.execute_queue()
        if not self.profilefollower.queue[0]:
            self.profilefollower.stop()
            self.profilefollower.modify_queue(heading=self.rotate_0, overwrite=True)
            self.profilefollower.execute_queue()
            self.next_state("rotate_towards_tower")

    @state
    def rotate_towards_tower(self, initial_call):
        if not self.profilefollower.queue[0]:
            print("ended_rotate")
            print("vision_x: %s" % (self.vision.x))
            if self.vision.x != 0.0:
                print("vision_detected")
                measure_trajectory = generate_trapezoidal_trajectory(
                        self.bno055.getHeading(),
                        0, self.bno055.getHeading()
                        + self.vision.derive_vision_angle(), 0,
                        self.rotate_velocity, self.rotate_accel_speed, -self.rotate_accel_speed)
                print(self.vision.x)
                print(self.vision.derive_vision_angle())
                print(measure_trajectory)
                print("vision_x %s, vision_angle %s, heading %s, heading_start %s, heading_end %s" % (
                    self.vision.x, self.vision.derive_vision_angle, self.bno055.getHeading(), measure_trajectory[0][0], measure_trajectory[-1][0]))
                self.profilefollower.modify_queue(heading=measure_trajectory, overwrite=True)
                self.profilefollower.execute_queue()
                # self.done()
            self.next_state("measure_position")

    @state
    def measure_position(self, initial_call):
        if not self.profilefollower.queue[0]:
            print("end_mpos")
            # now measure our position relative to the targets

            self.perpendicular_heading = 0
            if self.target == Targets.Right:
                self.perpendicular_heading = self.side_rotate_angle
            elif self.target == Targets.Left:
                self.perpendicular_heading = -self.side_rotate_angle

            r = (self.range_finder.getDistance() + self.center_to_front_bumper
                    - self.lidar_to_front_bumper)
            current_heading = self.bno055.getHeading()

            displacement_error = (
                    (r * math.sin(current_heading-self.perpendicular_heading))/
                    math.sin(math.pi-current_heading))
            print("vision_x: %s, range: %s, heading %s, displacement_error %s, raw_range %s" %
                    (self.vision.x, r, current_heading, displacement_error, self.range_finder.getDistance()))

            self.rotate_to_straight = generate_trapezoidal_trajectory(
                    current_heading, 0,
                    0, 0, self.rotate_velocity,
                    self.rotate_accel_speed, -self.rotate_accel_speed)
            self.displacement_correction = generate_trapezoidal_trajectory(
                    0, 0, displacement_error, 0,
                    Chassis.max_vel/3,
                    Chassis.max_acc, -Chassis.max_acc)
            self.profilefollower.modify_queue(heading=self.rotate_to_straight)
            self.profilefollower.execute_queue()
            self.next_state("rotate_straight")

    @state
    def rotate_straight(self, initial_call):
        # Drive from the range that vision and the lidar can detect the wall
        # to the wall itself
        if not self.profilefollower.queue[0]:
            self.profilefollower.modify_queue(0,
                    linear=self.displacement_correction)
            self.profilefollower.execute_queue()
            self.next_state("drive_align_segment")

    @state
    def drive_align_segment(self, initial_call):
        if not self.profilefollower.queue[0]:
            rotate_towards_wall = generate_trapezoidal_trajectory(
                    self.bno055.getHeading(), 0,
                    self.perpendicular_heading, 0, rotate_velocity,
                    self.rotate_accel_speed, -self.rotate_accel_speed)
            self.profilefollower.modify_queue(heading=rotate_towards_wall, overwrite=True)
            self.profilefollower.execute_queue()
            self.next_state("rotate_towards_peg")

    @state
    def rotate_towards_peg(self, initial_call):
        if not self.profilefollower.queue[0]:
            to_peg = generate_trapezoidal_trajectory(
                    0, 0, self.range_finder.getDistance()-self.lidar_to_front_bumper,
                    0,
                    Chassis.max_vel/3,
                    Chassis.max_acc, -Chassis.max_acc)
            self.profilefollower.modify_queue(to_peg,
                    linear=to_peg, overwrite=True)
            self.profilefollower.execute_queue()
            self.next_state("drive_to_wall")
            self.manipulategear.engage()

    @state
    def drive_to_wall(self, initial_call):
        if not self.profilefollower.queue[0]:
            # self.manipulategear.engage()
            pass

class LeftPeg(PegAutonomous):
    MODE_NAME = "Left Peg"

    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    chassis = Chassis

    def __init__(self):
        super().__init__(Targets.Left)

class CenterPeg(PegAutonomous):
    MODE_NAME = "Center Peg"

    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    chassis = Chassis

    def __init__(self):
        super().__init__(Targets.Center)

class RightPeg(PegAutonomous):
    MODE_NAME = "Right Peg"
    # DEFAULT = True

    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    chassis = Chassis

    def __init__(self):
        super().__init__(Targets.Right)
