import math

from magicbot.state_machine import AutonomousStateMachine, state
from networktables import NetworkTable

from automations.filters import RangeFilter, VisionFilter
from automations.manipulategear import ManipulateGear
from automations.profilefollower import ProfileFollower
from components.chassis import Chassis
from components.gears import GearAligner, GearDepositor
from components.range_finder import RangeFinder
from components.vision import Vision
from components.winch import Winch
from utilities.bno055 import BNO055
from utilities.profilegenerator import generate_trapezoidal_trajectory


class PegAutonomous(AutonomousStateMachine):
    # Injectables
    bno055 = BNO055
    chassis = Chassis
    gear_aligner = GearAligner
    gear_depositor = GearDepositor
    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    range_filter = RangeFilter
    range_finder = RangeFinder
    sd = NetworkTable
    vision = Vision
    vision_filter = VisionFilter
    winch = Winch

    centre_to_front_bumper = 0.49
    lidar_to_front_bumper = 0.36

    peg_range = 1.5
    dead_reckon_range = 2
    side_drive_forward_length = 2.54
    side_rotate_angle = math.pi/3.0
    rotate_accel_speed = 4 # rad*s^-2
    rotate_velocity = 4
    peg_align_tolerance = 0.15
    displace_velocity = Chassis.max_vel/3
    displace_accel = Chassis.max_acc
    displace_decel = Chassis.max_acc/4
    # rotate_accel_speed = 2 # rad*s^-2
    # rotate_velocity = 2

    def on_enable(self):
        super().on_enable()
        self.bno055.resetHeading()
        self.profilefollower.stop()
        self.gear_aligner.reset_position()
        self.gear_depositor.retract_gear()
        self.gear_depositor.lock_gear()
        self.init_trajectories()
        self.winch.enable_compressor()
        self.vision.enabled = True
        self.sd.putBoolean("log", True)

    @state(first=True)
    def drive_to_airship(self, initial_call):
        # Drive to a range where we can close the loop using vision, lidar and
        # gyro to close the loop on position
        if initial_call:
            displace = generate_trapezoidal_trajectory(
                0, 0, self.dr_displacement, 0, self.displace_velocity,
                self.displace_accel, -self.displace_decel,
                self.chassis.motion_profile_freq)
            self.profilefollower.modify_queue(heading=0, linear=displace)
            self.profilefollower.execute_queue()
        if not self.profilefollower.executing:
            self.next_state("rotate_towards_airship")

    @state
    def rotate_towards_airship(self, initial_call):
        if initial_call:
            rotate = generate_trapezoidal_trajectory(
                self.bno055.getHeading(), 0, self.perpendicular_heading, 0, self.rotate_velocity,
                self.rotate_accel_speed, -self.rotate_accel_speed,
                self.chassis.motion_profile_freq)
            self.profilefollower.modify_queue(heading=rotate, overwrite=True)
            self.logger.info("Rotate Start %s, End %s", rotate[0], rotate[-1])
            self.profilefollower.execute_queue()
        if not self.profilefollower.executing:
            self.next_state("rotate_towards_peg")

    @state
    def rotate_towards_peg(self, initial_call):
        if initial_call:
            if self.vision.x != 0.0:
                measure_trajectory = generate_trapezoidal_trajectory(
                    self.bno055.getHeading(), 0,
                    self.bno055.getHeading() + self.vision.derive_vision_angle(), 0,
                    self.rotate_velocity, self.rotate_accel_speed, -self.rotate_accel_speed/2,
                    Chassis.motion_profile_freq)
                self.logger.info("vision_x %s, vision_angle %s, heading %s, heading_start %s, heading_end %s",
                    self.vision.x, self.vision.derive_vision_angle(), self.bno055.getHeading(), measure_trajectory[0][0], measure_trajectory[-1][0])
                self.profilefollower.modify_queue(heading=measure_trajectory, overwrite=True)
                self.profilefollower.execute_queue()
                # self.done()
        if not self.profilefollower.executing:
            self.next_state("drive_to_wall")

    @state
    def drive_to_wall(self, initial_call):
        if initial_call:
            self.profilefollower.stop()
            peg_range = self.peg_range - self.centre_to_front_bumper + 0.3
            r = self.range_filter.range
            self.logger.info("DRIVE WALL RANGE: %s", self.range_finder.getDistance())
            self.logger.info("DRIVE WALL FILTER RANGE: %s", self.range_filter.range)
            # 40 is range finder max distance, better failure mode than inf or really small
            if not math.isfinite(r):
                r = 40
            elif r < 0.5:
                r = 40
            to_peg = None
            if r > self.dead_reckon_range:
                self.logger.info("DEAD RECKON AUTO")
                to_peg = generate_trapezoidal_trajectory(
                    0, 0, peg_range + 0.1, 0, self.displace_velocity,
                    self.displace_accel, -self.displace_decel,
                    Chassis.motion_profile_freq)
            else:
                self.logger.info("RANGE AUTO")
                to_peg = generate_trapezoidal_trajectory(0, 0,
                    self.range_finder.getDistance() - self.lidar_to_front_bumper + 0.1,
                    0, self.displace_velocity, self.displace_accel, -self.displace_decel,
                    Chassis.motion_profile_freq)
            self.profilefollower.modify_queue(self.bno055.getHeading(),
                linear=to_peg, overwrite=True)
            self.profilefollower.execute_queue()
            self.manipulategear.engage()

        if not self.profilefollower.executing:
            self.next_state("deploying_gear")

    @state
    def deploying_gear(self):
        if self.manipulategear.current_state == "forward_open":
            # self.next_state("roll_back")
            self.done()
        # elif self.manipulategear.current_state == "backward_open":
        #     self.next_state("roll_back")
        elif self.manipulategear.current_state != "forward_closed":
            self.manipulategear.engage(initial_state="forward_closed", force=True)

    @state
    def roll_back(self, initial_call):
        if initial_call:
            self.profilefollower.stop()
            roll_back = generate_trapezoidal_trajectory(
                0, 0, -2, 0, self.displace_velocity, self.displace_accel/2,
                -self.displace_decel, Chassis.motion_profile_freq)
            self.profilefollower.modify_queue(self.bno055.getHeading(),
                linear=roll_back, overwrite=True)
            self.profilefollower.execute_queue()
        elif not self.profilefollower.executing:
            self.done()

    def done(self):
        super().done()
        self.vision.enabled = False


class LeftPeg(PegAutonomous):
    MODE_NAME = "Left Peg"

    def init_trajectories(self):
        self.perpendicular_heading = -self.side_rotate_angle
        self.dr_displacement = self.side_drive_forward_length - self.centre_to_front_bumper


class CentrePeg(PegAutonomous):
    MODE_NAME = "Centre Peg"

    centre_airship_distance = 2.85
    peg_range = centre_airship_distance
    dead_reckon_range = 4

    def init_trajectories(self):
        self.perpendicular_heading = 0
        self.dr_displacement = self.centre_airship_distance/2 - self.centre_to_front_bumper

    @state(first=True)
    def drive_to_airship(self):
        # Override the initial state here to immediately go to the state we want
        # Use next_state_now to avoid wasting a control loop iteration
        self.next_state_now("drive_to_wall")


class RightPeg(PegAutonomous):
    MODE_NAME = "Right Peg"

    def init_trajectories(self):
        self.perpendicular_heading = self.side_rotate_angle
        self.dr_displacement = self.side_drive_forward_length - self.centre_to_front_bumper
