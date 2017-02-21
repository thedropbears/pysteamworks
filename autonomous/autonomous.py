from automations.manipulategear import ManipulateGear
from automations.profilefollower import ProfileFollower
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

    center_tower_distance = 2.93
    side_drive_forward_length = 2.54
    side_rotate_angle = math.pi/3.0
    rotate_accel_speed = 2 # rad*s^-2
    rotate_velocity = 2
    peg_align_tolerance = 0.15
    displace_velocity = Chassis.max_vel
    displace_accel = Chassis.max_acc
    displace_decel = Chassis.max_acc/3
    # rotate_accel_speed = 2 # rad*s^-2
    # rotate_velocity = 2

    def __init__(self, target=Targets.Center):
        super().__init__()
        self.target = target

    def generate_trajectories(self):
        if self.target == Targets.Center:
            self.perpendicular_heading = 0
            self.displace_0 = generate_trapezoidal_trajectory(
                    0, 0, self.center_tower_distance-(2*self.center_to_front_bumper),
                    0, self.displace_velocity,
                    self.displace_accel, -self.displace_decel)
            self.rotate_0 = []
        elif self.target == Targets.Left:
            self.perpendicular_heading = -self.side_rotate_angle
            self.displace_0 = generate_trapezoidal_trajectory(
                    0, 0, self.side_drive_forward_length-self.center_to_front_bumper,
                    0, self.displace_velocity,
                    self.displace_accel, -self.displace_decel)
            self.rotate_0 = generate_trapezoidal_trajectory(
                    0, 0, -self.side_rotate_angle, 0, self.rotate_velocity,
                    self.rotate_accel_speed, -self.rotate_accel_speed/2)
        else:
            self.perpendicular_heading = self.side_rotate_angle
            self.displace_0 = generate_trapezoidal_trajectory(
                    0, 0, self.side_drive_forward_length-self.center_to_front_bumper, 0,
                    self.displace_velocity,
                    self.displace_accel, -self.displace_decel)
            self.rotate_0 = generate_trapezoidal_trajectory(
                    0, 0, self.side_rotate_angle, 0, self.rotate_velocity,
                    self.rotate_accel_speed, -self.rotate_accel_speed/2)

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
            self.profilefollower.modify_queue(heading=0,
                    linear=self.displace_0)

            self.profilefollower.execute_queue()
        if not self.profilefollower.queue[0]:
            self.profilefollower.stop()
            if not self.target == Targets.Center:
                self.profilefollower.modify_queue(heading=self.rotate_0, overwrite=True)
                print("Rotate Start %s, Rotate End %s" % (self.rotate_0[0], self.rotate_0[-1]))
                self.profilefollower.execute_queue()
                self.next_state("rotate_towards_tower")
            else:
                self.next_state("rotate_towards_peg")


    @state
    def rotate_towards_tower(self, initial_call):
        if not self.profilefollower.queue[0]:
            if self.vision.x != 0.0:
                measure_trajectory = generate_trapezoidal_trajectory(
                        self.bno055.getHeading(),
                        0, self.bno055.getHeading()
                        + self.vision.derive_vision_angle(), 0,
                        self.rotate_velocity, self.rotate_accel_speed, -self.rotate_accel_speed/2)
                print("vision_x %s, vision_angle %s, heading %s, heading_start %s, heading_end %s" % (
                    self.vision.x, self.vision.derive_vision_angle(), self.bno055.getHeading(), measure_trajectory[0][0], measure_trajectory[-1][0]))
                self.profilefollower.modify_queue(heading=measure_trajectory, overwrite=True)
                self.profilefollower.execute_queue()
                # self.done()
            self.next_state("rotate_towards_peg")

    @state
    def measure_position(self, initial_call):
        if initial_call:
            if self.vision.x != 0.0:
                measure_trajectory = generate_trapezoidal_trajectory(
                        self.bno055.getHeading(),
                        0, self.bno055.getHeading()
                        + self.vision.derive_vision_angle(), 0,
                        self.rotate_velocity, self.rotate_accel_speed, -self.rotate_accel_speed/2)
                print("vision_x %s, vision_angle %s, heading %s, heading_start %s, heading_end %s" % (
                    self.vision.x, self.vision.derive_vision_angle(), self.bno055.getHeading(), measure_trajectory[0][0], measure_trajectory[-1][0]))
                self.profilefollower.modify_queue(heading=measure_trajectory, overwrite=True)
                self.profilefollower.execute_queue()
        elif not self.profilefollower.queue[0]:
            print("end_mpos")
            # now measure our position relative to the targets


            r = (self.range_finder.getDistance() + self.center_to_front_bumper
                    - self.lidar_to_front_bumper)
            current_heading = self.bno055.getHeading()

            displacement_error = -(
                    (r * math.sin(current_heading-self.perpendicular_heading))/
                    math.sin(math.pi-current_heading))
            print("vision_x: %s, range: %s, heading %s, displacement_error %s, raw_range %s" %
                    (self.vision.x, r, current_heading, displacement_error, self.range_finder.getDistance()))

            self.rotate_to_correct = generate_trapezoidal_trajectory(
                    current_heading, 0,
                    0, 0, self.rotate_velocity,
                    self.rotate_accel_speed, -self.rotate_accel_speed/2)
            self.displacement_correction = generate_trapezoidal_trajectory(
                    0, 0, displacement_error, 0,
                    self.displace_velocity,
                    self.displace_accel, -self.displace_decel)
            if abs(displacement_error) < self.peg_align_tolerance:
                print("WITHIN TOLERANCE, SKIPPING...")
                self.next_state("rotate_towards_peg")
            else:
                self.profilefollower.modify_queue(heading=self.rotate_to_correct)
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
                    self.perpendicular_heading, 0, self.rotate_velocity,
                    self.rotate_accel_speed, -self.rotate_accel_speed/2)
            self.profilefollower.modify_queue(heading=rotate_towards_wall, overwrite=True)
            self.profilefollower.execute_queue()
            self.next_state("rotate_towards_peg")

    @state
    def rotate_towards_peg(self, initial_call):
        if not self.profilefollower.queue[0]:
            to_peg = generate_trapezoidal_trajectory(
                    0, 0, self.range_finder.getDistance()-self.lidar_to_front_bumper,
                    0,
                    self.displace_velocity,
                    self.displace_accel, -self.displace_decel*2)
            self.profilefollower.modify_queue(self.perpendicular_heading,
                    linear=to_peg, overwrite=True)
            self.profilefollower.execute_queue()
            self.next_state("drive_to_wall")
            self.manipulategear.engage()

    @state
    def drive_to_wall(self, initial_call):
        if not self.manipulategear.is_executing:
            roll_back = generate_trapezoidal_trajectory(
                    0, 0, -1, 0, self.displace_velocity,
                    self.displace_accel, -self.displace_decel)
            self.profilefollower.modify_queue(self.perpendicular_heading,
                    linear=roll_back, overwrite=True)
            self.profilefollower.execute_queue()
            self.next_state("roll_back")
    @state
    def roll_back(self):
        if not self.profilefollower.queue[0]:
            self.done()

    def done(self):
        super().done()
        self.vision.vision_mode = False


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
