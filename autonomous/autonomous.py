from automations.manipulategear import ManipulateGear
from automations.profilefollower import ProfileFollower
from automations.vision_filter import VisionFilter
from automations.range_filter import RangeFilter
from components.chassis import Chassis
from components.bno055 import BNO055
from components.vision import Vision
from components.gearalignmentdevice import GearAlignmentDevice
from components.geardepositiondevice import GearDepositionDevice
from components.range_finder import RangeFinder
from magicbot.state_machine import AutonomousStateMachine, state
from utilities.profilegenerator import generate_interpolation_trajectory
from utilities.profilegenerator import generate_trapezoidal_trajectory
from components.winch import Winch

import math

class Targets:
    Left = 0
    Centre = 1
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
    winch = Winch
    vision_filter = VisionFilter
    range_filter = RangeFilter

    centre_to_front_bumper = 0.49
    lidar_to_front_bumper = 0.36

    centre_airship_distance = 2.85
    side_drive_forward_length = 2.54
    side_rotate_angle = math.pi/3.0
    rotate_accel_speed = 2 # rad*s^-2
    rotate_velocity = 3
    peg_align_tolerance = 0.15
    displace_velocity = Chassis.max_vel
    displace_accel = Chassis.max_acc
    displace_decel = Chassis.max_acc/3
    # rotate_accel_speed = 2 # rad*s^-2
    # rotate_velocity = 2

    def __init__(self, target=Targets.Centre):
        super().__init__()
        self.target = target

    def generate_trajectories(self):
        if self.target is Targets.Left:
            self.perpendicular_heading = -self.side_rotate_angle
            self.dr_displacement = self.side_drive_forward_length-self.centre_to_front_bumper
        elif self.target is Targets.Right:
            self.perpendicular_heading = self.side_rotate_angle
            self.dr_displacement = self.side_drive_forward_length-self.centre_to_front_bumper
        else:
            self.perpendicular_heading = 0
            self.dr_displacement = self.centre_airship_distance/2-self.centre_to_front_bumper

    def on_enable(self):
        super().on_enable()
        self.bno055.resetHeading()
        self.profilefollower.stop()
        self.gearalignmentdevice.reset_position()
        self.geardepositiondevice.retract_gear()
        self.geardepositiondevice.lock_gear()
        self.generate_trajectories()
        self.winch.enable_compressor()
        self.vision.vision_mode = True

    @state(first=True)
    def drive_to_airship(self, initial_call):
        # Drive to a range where we can close the loop using vision, lidar and
        # gyro to close the loop on position
        if initial_call:
            self.vision.vision_mode = True
            if self.target == Targets.Centre:
                self.next_state("drive_to_wall")
                return
            displace = generate_trapezoidal_trajectory(
                    0, 0, self.dr_displacement,
                    0, self.displace_velocity,
                    self.displace_accel, -self.displace_decel)
            self.profilefollower.modify_queue(heading=0, linear=displace)
            self.profilefollower.execute_queue()
        if not self.profilefollower.queue[0]:
            if self.target is Targets.Centre:
                self.next_state("rotate_towards_peg")
            else:
                self.next_state("rotate_towards_airship")


    @state
    def rotate_towards_airship(self, initial_call):
        if initial_call:
            rotate = generate_trapezoidal_trajectory(
                    self.bno055.getHeading(), 0, self.perpendicular_heading, 0, self.rotate_velocity,
                    self.rotate_accel_speed, -self.rotate_accel_speed)
            self.profilefollower.modify_queue(heading=rotate, overwrite=True)
            print("Rotate Start %s, Rotate End %s" % (rotate[0], rotate[-1]))
            self.profilefollower.execute_queue()
        if not self.profilefollower.queue[0]:
            self.next_state("rotate_towards_peg")

    @state
    def measure_position(self, initial_call):
        if initial_call:
            # use unfiltered vision here
            if self.vision.x != 0.0:
                measure_trajectory = generate_trapezoidal_trajectory(
                        self.bno055.getHeading(),
                        0, self.bno055.getHeading()
                        + self.vision_filter.angle, 0,
                        self.rotate_velocity, self.rotate_accel_speed, -self.rotate_accel_speed/2)
                print(self.vision.derive_vision_angle())
                print("vision_x %s, vision_angle %s, heading %s, heading_start %s, heading_end %s" % (
                    self.vision.x, self.vision.derive_vision_angle(), self.bno055.getHeading(), measure_trajectory[0][0], measure_trajectory[-1][0]))
                self.profilefollower.modify_queue(heading=measure_trajectory, overwrite=True)
                self.profilefollower.execute_queue()
        if not self.profilefollower.queue[0]:
            # now measure our position relative to the targets
            r = (self.range_filter.range + self.centre_to_front_bumper
                 - self.lidar_to_front_bumper)
            current_heading = self.bno055.getHeading()+self.vision.derive_vision_angle()

            self.displacement_error = -(
                    (r * math.sin(current_heading-self.perpendicular_heading))/
                    math.sin(math.pi-current_heading))
            print("vision_x: %s, range: %s, heading %s, displacement_error %s, raw_range %s" %
                    (self.vision.x, r, current_heading, self.displacement_error, self.range_finder.getDistance()))

            if abs(self.displacement_error) < self.peg_align_tolerance:
                print("WITHIN TOLERANCE, SKIPPING...")
                self.next_state("rotate_towards_peg")
            else:
                self.next_state("rotate_straight")

    @state
    def rotate_straight(self, initial_call):
        # Drive from the range that vision and the lidar can detect the wall
        # to the wall itself
        if initial_call:
            current_heading = self.bno055.getHeading()
            self.rotate_to_correct = generate_trapezoidal_trajectory(
                    current_heading, 0,
                    0, 0, self.rotate_velocity,
                    self.rotate_accel_speed, -self.rotate_accel_speed/2)
            self.profilefollower.modify_queue(heading=self.rotate_to_correct)
            self.profilefollower.execute_queue()
        if not self.profilefollower.queue[0]:
            self.next_state("drive_align_segment")

    @state
    def drive_align_segment(self, initial_call):
        if initial_call:
            displacement_correction = generate_trapezoidal_trajectory(
                    0, 0, self.displacement_error, 0,
                    self.displace_velocity,
                    self.displace_accel, -self.displace_decel)
            self.profilefollower.modify_queue(0,
                    linear=displacement_correction)
            self.profilefollower.execute_queue()
        if not self.profilefollower.queue[0]:
            self.next_state("rotate_towards_airship")

    @state
    def rotate_towards_peg(self, initial_call):
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
                # self.done()
        if not self.profilefollower.queue[0]:
            self.next_state("drive_to_wall")

    @state
    def drive_to_wall(self, initial_call):
        if initial_call:
            self.profilefollower.stop()
            peg_range = 1.5-self.centre_to_front_bumper
            if self.target == Targets.Centre:
                peg_range = (self.centre_airship_distance)-self.centre_to_front_bumper
            # r = self.range_filter.range
            r = self.range_filter.range
            print("AUTO DRIVE WALL RANGE: %s" % (self.range_finder.getDistance()))
            print("AUTO DRIVE WALL FILTER RANGE: %s" % (self.range_filter.range))
            # 40 is range finder max distance, better failure mode than inf or really small
            if float(r) is float('inf') or float(r) is float('nan'):
                r = 40
            elif r < 0.5:
                r = 40
            to_peg = None
            if (r>2 and self.target != Targets.Centre) or (r>4 and self.target == Targets.Centre):
                print("DEAD RECKON AUTO")
                to_peg = generate_trapezoidal_trajectory(
                        0, 0, peg_range+0.1,
                        0,
                        self.displace_velocity,
                        self.displace_accel, -self.displace_decel)
            else:
                print("RANGE AUTO")
                to_peg = generate_trapezoidal_trajectory(
                        0, 0, self.range_finder.getDistance()-self.lidar_to_front_bumper+0.1,
                        0,
                        self.displace_velocity,
                        self.displace_accel, -self.displace_decel)
            self.profilefollower.modify_queue(self.bno055.getHeading(),
                    linear=to_peg, overwrite=True)
            self.profilefollower.execute_queue()
            self.manipulategear.engage()
        elif self.manipulategear.current_state == "forward_open":
            self.done()
            # self.next_state("roll_back")

        if not self.profilefollower.queue[0]:
            if not (self.manipulategear.current_state == "forward_open" or self.manipulategear.current_state == "forward_closed"):
                self.manipulategear.engage(initial_state="forward_closed", force=True)
        # elif self.manipulategear.current_state == "backward_open":
        #     self.next_state("roll_back")

    @state
    def roll_back(self, initial_call):
        if initial_call:
            self.profilefollower.stop()
            roll_back = generate_trapezoidal_trajectory(
                    0, 0, -2, 0, self.displace_velocity,
                    self.displace_accel/2, -self.displace_decel)
            self.profilefollower.modify_queue(self.bno055.getHeading(),
                    linear=roll_back, overwrite=True)
            self.profilefollower.execute_queue()
        elif not self.profilefollower.queue[0]:
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

class CentrePeg(PegAutonomous):
    MODE_NAME = "Centre Peg"

    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    chassis = Chassis

    def __init__(self):
        super().__init__(Targets.Centre)

class RightPeg(PegAutonomous):
    MODE_NAME = "Right Peg"
    # DEFAULT = True

    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    chassis = Chassis

    def __init__(self):
        super().__init__(Targets.Right)
