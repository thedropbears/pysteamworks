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
    lidar_to_front_bumper = 0.67

    center_tower_distance = 3
    side_drive_forward_length = 1
    side_rotate_angle = math.pi/3.0

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
        self.bno055.resetHeading()

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
                        2, 2, -2)
                print(self.vision.x)
                print(self.vision.derive_vision_angle())
                print(measure_trajectory)
                self.profilefollower.modify_queue(heading=measure_trajectory, overwrite=True)
                self.profilefollower.execute_queue()
                # self.done()
            self.next_state("measure_position")

    @state
    def measure_position(self, initial_call):
        if not self.profilefollower.queue[0]:
            print("end_mpos")
            # now measure our position relative to the targets

            # the angle that we should be at
            self.t_d = 0
            if self.target == Targets.Center:
                self.t_d = 0
            elif self.target == Targets.Left:
                self.t_d = -self.side_rotate_angle
            elif self.target == Targets.Right:
                self.t_d = self.side_rotate_angle
            # the error from the heading we should be at to be facing
            # towards the wall
            t_e = self.bno055.getHeading() - self.t_d
            r_m = (self.range_finder.getDistance() - self.lidar_to_front_bumper
                    + self.center_to_front_bumper)
            # how far back we are from the wall
            r_x = r_m * math.cos(t_e)
            # how far to the side of the peg we are
            r_y = r_m * math.sin(t_e)

            d_delta_1 = math.sqrt(
                (r_x/2)**2 + r_m**2 - 2*(r_x/2)*r_m*math.cos(t_e))
            print(r_x*math.sin(t_e)/2*d_delta_1)
            t_delta_1 = math.asin(r_x*math.sin(t_e)/(2*d_delta_1))
            # t_delta_2 = -math.asin(r_m*math.sin(t_e)/d_delta_1)+math.pi
            # t_delta_2 = -math.asin(r_m*math.sin(t_e)/d_delta_1)
            d_delta_2 = r_x/2
            # print("t_e: %s, r_m %s, r_x %s, r_y %s, t_delta_1 %s, d_delta_1 %s, t_delta_2 %s, d_delta_2 %s" %
            #         (t_e, r_m, r_x, r_y, t_delta_1, d_delta_1, t_delta_2, t_delta_2))
            self.seg_1_heading = self.bno055.getHeading()+t_delta_1
            # self.seg_2_heading = self.bno055.getHeading()+t_delta_1+t_delta_2
            self.rotate_1 = generate_trapezoidal_trajectory(
                    self.seg_1_heading, 0,
                    self.bno055.getHeading()+t_delta_1, 0, 2,
                    2, -2)
            self.displace_1 = generate_trapezoidal_trajectory(
                    0, 0, d_delta_1, 0,
                    Chassis.max_vel/2,
                    Chassis.max_acc, -Chassis.max_acc)
            # TODO: should we be resetting the encoders mid way and treat
            # as two separate trajectories?
            # self.rotate_2 = generate_trapezoidal_trajectory(
            #         self.bno055.getHeading()+t_delta_1, 0,
            #         self.bno055.getHeading()+t_delta_1+t_delta_2, 0, 2,
            #         2, -2)
            # self.displace_2 = generate_trapezoidal_trajectory(
            #         d_delta_1, 0, d_delta_1+d_delta_2, 0,
            #         Chassis.max_vel/2,
            #         Chassis.max_acc, -Chassis.max_acc)
            self.profilefollower.modify_queue(heading=self.rotate_1)
            self.profilefollower.execute_queue()
            self.next_state("rotate_to_align_segment")

    @state
    def rotate_to_align_segment(self, initial_call):
        # Drive from the range that vision and the lidar can detect the wall
        # to the wall itself
        if not self.profilefollower.queue[0]:
            self.profilefollower.modify_queue(self.seg_1_heading,
                    linear=self.displace_1)
            self.profilefollower.execute_queue()
            self.next_state("drive_align_segment")

    @state
    def drive_align_segment(self, initial_call):
        if not self.profilefollower.queue[0]:
            rotate_towards_wall = generate_trapezoidal_trajectory(
                    self.bno055.getHeading(), 0,
                    self.t_d, 0, 2,
                    2, -2)
            self.profilefollower.modify_queue(heading=rotate_towards_wall, overwrite=True)
            self.profilefollower.execute_queue()
            self.next_state("rotate_towards_peg")

    @state
    def rotate_towards_peg(self, initial_call):
        if not self.profilefollower.queue[0]:
            to_peg = generate_trapezoidal_trajectory(
                    0, 0, self.range_finder.getDistance()-self.lidar_to_front_bumper,
                    0,
                    Chassis.max_vel/2,
                    Chassis.max_acc, -Chassis.max_acc)
            self.profilefollower.modify_queue(self.t_d,
                    linear=to_peg, overwrite=True)
            self.profilefollower.execute_queue()
            self.next_state("drive_to_wall")

    @state
    def drive_to_wall(self, initial_call):
        if not self.profilefollower.queue[0]:
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

    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    chassis = Chassis

    def __init__(self):
        super().__init__(Targets.Center)

class RightPeg(PegAutonomous):
    MODE_NAME = "Right Peg"
    DEFAULT = True

    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    chassis = Chassis

    def __init__(self):
        super().__init__(Targets.Right)
