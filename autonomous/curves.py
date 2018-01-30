import math

import numpy as np
from magicbot import MagicRobot
from magicbot.state_machine import AutonomousStateMachine, state
from networktables import NetworkTable

from automations.filters import VisionFilter
from automations.manipulategear import ManipulateGear
from automations.profilefollower import ProfileFollower
from components.chassis import Chassis
from components.gears import GearAligner, GearDepositor
from components.range_finder import RangeFinder
from components.vision import Vision
from utilities.bno055 import BNO055
from utilities.profilegenerator import generate_cubic_trajectory


class PegAutonomous(AutonomousStateMachine):
    # Injectables
    bno055 = BNO055
    chassis = Chassis
    gear_aligner = GearAligner
    gear_depositor = GearDepositor
    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    range_finder = RangeFinder
    sd = NetworkTable
    vision = Vision
    vision_filter = VisionFilter

    centre_to_front_bumper = 0.49
    lidar_to_front_bumper = 0.36

    centre_airship_distance = 2.93
    side_drive_forward_distance = 2.54 - centre_to_front_bumper
    side_to_wall_distance = 1.62-centre_to_front_bumper+0.4  # .4 added in order to drive hard into wall
    side_rotate_angle = math.pi/3.0
    rotate_radius = 0.7
    rotate_linear_velocity = 1
    delta_s = abs(rotate_radius*math.tan(side_rotate_angle/2))
    rotate_arc_length = rotate_radius * side_rotate_angle

    dt = MagicRobot.control_loop_wait_time

    def generate_trajectories(self):
        t1 = 1.5  # s, time for segment 1
        rotate_time = self.rotate_arc_length/self.rotate_linear_velocity
        t3 = 2.5  # s, time for segment 3
        distance_keypoints = [
            (0, 0, 0),
            (t1, self.side_drive_forward_distance-self.delta_s, self.rotate_linear_velocity),
            (t1 + rotate_time, self.side_drive_forward_distance - self.delta_s + self.rotate_arc_length, self.rotate_linear_velocity),
            (t1 + rotate_time + t3, self.side_drive_forward_distance + self.rotate_arc_length + self.side_to_wall_distance - 2*self.delta_s, 0)
        ]
        self.logger.info("ds %s, drive_forward_dist-ds %s, rotate_arc_length %s, rotate_tm %s",
            self.delta_s, self.side_drive_forward_distance-self.delta_s, self.rotate_arc_length, rotate_time)
        self.gear_mech_on = int(t3/self.dt) # Segments left when gear mech enabled
        heading_rate = self.perpendicular_heading / rotate_time
        self.heading_trajectory = [(0, 0, 0)] * int(t1/self.dt)
        self.heading_trajectory += [(x, heading_rate, 0) for x in np.arange(0, rotate_time, self.dt) * heading_rate]
        self.heading_trajectory += [(self.perpendicular_heading, 0, 0)] * int(t3/self.dt)
        self.distance_trajectory = generate_cubic_trajectory(distance_keypoints, self.dt)
        # print("Distance Traj Len %s, Heading Traj Len %s" % (len(self.distance_trajectory), len(self.heading_trajectory)))
        # print("Distance trajectory", self.distance_trajectory[int(t1/self.dt)])
        # print("Heading trajectory", self.heading_trajectory[int(t1/self.dt)])
        # print("Distance trajectory", self.distance_trajectory[int(t1/self.dt)+1])
        # print("Heading trajectory", self.heading_trajectory[int(t1/self.dt)+1])

    def on_enable(self):
        super().on_enable()
        self.bno055.resetHeading()
        self.profilefollower.stop()
        self.gear_aligner.reset_position()
        self.gear_depositor.retract_gear()
        self.gear_depositor.lock_gear()
        self.generate_trajectories()
        self.sd.putBoolean("log", True)

    @state(first=True)
    def drive_to_airship(self, initial_call):
        self.sd.putNumber("left_speed_error", self.chassis.drive_motor_a.getClosedLoopError())
        self.sd.putNumber("right_speed_error", self.chassis.drive_motor_c.getClosedLoopError())
        # Drive to a range where we can close the loop using vision, lidar and
        # gyro to close the loop on position
        if initial_call:
            self.vision.enabled = True
            self.profilefollower.modify_queue(heading=self.heading_trajectory,
                    linear=self.distance_trajectory)
            self.profilefollower.execute_queue()
        if len(self.profilefollower.linear_queue) <= self.gear_mech_on and not self.manipulategear.is_executing:
            self.vision_filter.reset()
            self.manipulategear.engage()
        if not self.profilefollower.executing:
            self.next_state("deploying_gear")

    @state
    def deploying_gear(self, initial_call):
        gear_state = self.manipulategear.current_state
        if initial_call and gear_state == "align_peg":
            self.manipulategear.engage(initial_state="forward_closed", force=True)
        elif gear_state == "forward_open":
            self.done()

    def done(self):
        super().done()
        self.vision.enabled = False


class LeftPegCurves(PegAutonomous):
    MODE_NAME = "Left Peg Curves"
    #DISABLED = True

    def __init__(self):
        super().__init__()
        self.perpendicular_heading = -self.side_rotate_angle


class RightPegCurves(PegAutonomous):
    MODE_NAME = "Right Peg Curves"
    #DISABLED = True

    def __init__(self):
        super().__init__()
        self.perpendicular_heading = self.side_rotate_angle


class CentrePegCurves(PegAutonomous):
    MODE_NAME = "Centre Peg Curves"
    #DISABLED = True

    def generate_trajectories(self):
        t1 = 1.5
        distance_keypoints = [(0, 0, 0),
            (t1, self.centre_airship_distance - 2*self.centre_to_front_bumper, 0)]
        self.gear_mech_on = t1/self.dt # Segments left when gear mech enabled
        self.heading_trajectory = [(0, 0, 0)] * int(t1/self.dt)
        self.distance_trajectory = generate_cubic_trajectory(distance_keypoints, self.dt)
