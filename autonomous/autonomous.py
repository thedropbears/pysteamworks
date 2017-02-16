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

    center_tower_distance = 1
    side_drive_forward_length = 3
    side_rotate_angle = math.pi/6.0

    def __init__(self, target=Targets.Center):
        super().__init__()
        self.target = target

        if target == Targets.Center:
            self.displace_0 = generate_trapezoidal_trajectory(
                    0, 0, self.center_tower_distance, 0, Chassis.max_vel/2,
                    Chassis.max_acc, -Chassis.max_acc)
            self.rotate_0 = []
        elif target == Targets.Left:
            self.displace_0 = generate_trapezoidal_trajectory(
                    0, 0, self.side_drive_forward_length, 0,
                    Chassis.max_vel/2,
                    Chassis.max_acc, -Chassis.max_acc)
            self.rotate_0 = generate_trapezoidal_trajectory(
                    0, 0, -self.side_rotate_angle, 0, 2,
                    2, -2)
        else:
            self.displace_0 = generate_trapezoidal_trajectory(
                    0, 0, self.side_drive_forward_length, 0,
                    Chassis.max_vel/2,
                    Chassis.max_acc, -Chassis.max_acc)
            self.rotate_0 = generate_trapezoidal_trajectory(
                    0, 0, self.side_rotate_angle, 0, 2,
                    2, -2)

    def on_enable(self):
        super().on_enable()

    @state(first=True)
    def drive_to_airship(self, initial_call):
        # Drive to a range where we can close the loop using vision, lidar and
        # gyro to close the loop on position
        if initial_call:
            self.profilefollower.modify_queue(linear=self.displace_0)
            if not self.target == Targets.Center:
                self.profilefollower.modify_queue(heading=self.rotate_0)

            self.profilefollower.execute_queue()
        if not self.profilefollower.queue[0]:
            if self.vision.x != 0.0:
                measure_trajectory = generate_trapezoidal_trajectory(
                        self.bno055.getHeading(),
                        0, self.bno055.getHeading()
                        + self.vision.derive_vision_ange(), 0,
                        2, 2, -2)
                self.profilefollower.modify_queue(heading=measure_trajectory)
            self.next_state("measure_position")

    @state
    def measure_position(self, initial_call):
        if not self.profilefollower.queue[0]:
            # now measure our position relative to the targets

            # the angle that we should be at
            t_d = 0
            if self.target == Targets.Center:
                theta_d = 0
            elif self.target == Targets.Left:
                theta_d = -self.side_rotate_angle
            elif self.target == Targets.Right:
                theta_d = self.side_rotate_angle
            # the error from the heading we should be at to be facing
            # towards the wall
            t_e = self.bno055.getHeading() - t_d
            r_m = self.range_finder.getDistance()
            # how far back we are from the wall
            r_x = r_m * math.cos(t_e)
            # how far to the side of the peg we are
            r_y = r_m * math.sin(t_e)

            d_delta_1 = math.sqrt(
                (r_x/2)**2 + r_m**2 - 2*(r_x/2)*r_m*math.cos(t_e))
            t_delta_1 = math.asin(r_x*math.sin(t_e)/2*d_delta_1)
            t_delta_2 = -math.asin(r_m*math.sin(t_e)/d_delta_1)+math.pi
            d_delta_2 = r_x/2
            self.rotate_1 = generate_trapezoidal_trajectory(
                    self.bno055.getHeading(), 0,
                    self.bno055.getHeading()+t_delta_1, 0, 2,
                    2, -2)
            self.displace_1 = generate_trapezoidal_trajectory(
                    0, 0, d_delta_1, 0,
                    Chassis.max_vel/2,
                    Chassis.max_acc, -Chassis.max_acc)
            # TODO: should we be resetting the encoders mid way and treat
            # as two separate trajectories?
            self.rotate_2 = generate_trapezoidal_trajectory(
                    self.bno055.getHeading()+t_delta_1, 0,
                    self.bno055.getHeading()+t_delta_1+t_delta_2, 0, 2,
                    2, -2)
            self.displace_2 = generate_trapezoidal_trajectory(
                    d_delta_1, 0, d_delta_1+d_delta_2, 0,
                    Chassis.max_vel/2,
                    Chassis.max_acc, -Chassis.max_acc)
            self.profilefollower.modify_queue(heading=self.rotate_1)
            self.profilefollower.modify_queue(linear=self.displace_1)
            self.profilefollower.modify_queue(heading=self.rotate_2)
            self.profilefollower.modify_queue(linear=self.displace_2)
            self.next_state("drive_to_wall")

    @state
    def drive_to_wall(self, initial_call):
        # Drive from the range that vision and the lidar can detect the wall
        # to the wall itself
        if not self.profilefollower.queue[0]:
            # TODO: a final check to ensure that we are at the right range?
            self.manipulategear.engage()

class LeftPeg(PegAutonomous):
    MODE_NAME = "Left Peg"

    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    chassis = Chassis

    def __init__(self):
        super().__init__(Targets.Left)

class CenterPeg(PegAutonomous):
    MODE_NAME = "Center Peg"
    DEFAULT = True

    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    chassis = Chassis

    def __init__(self):
        super().__init__(Targets.Center)

class RightPeg(PegAutonomous):
    MODE_NAME = "Right Peg"

    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    chassis = Chassis

    def __init__(self):
        super().__init__(Targets.Right)
