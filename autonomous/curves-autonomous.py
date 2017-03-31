from automations.manipulategear import ManipulateGear
from automations.profilefollower import ProfileFollower
from components.chassis import Chassis
from components.bno055 import BNO055
from components.vision import Vision
from components.gearalignmentdevice import GearAlignmentDevice
from components.geardepositiondevice import GearDepositionDevice
from components.range_finder import RangeFinder
from magicbot.state_machine import AutonomousStateMachine, state
from utilities.profilegenerator import cubic_generator
from networktables import NetworkTable
import numpy as np

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
    sd = NetworkTable

    centre_to_front_bumper = 0.49
    lidar_to_front_bumper = 0.36

    centre_airship_distance = 2.93
    side_drive_forward_distance = 2.54 - centre_to_front_bumper
    side_to_wall_distance = 1.5-centre_to_front_bumper
    side_rotate_angle = math.pi/3.0
    rotate_radius = 0.7
    rotate_linear_velocity = 1.5
    delta_s = abs(rotate_radius*math.tan(side_rotate_angle/2))
    rotate_arc_length = rotate_radius * side_rotate_angle

    dt = 0.02


    def __init__(self, target=Targets.Centre):
        super().__init__()
        self.target = target

    def generate_trajectories(self):
        if self.target is Targets.Centre:
            t1 = 1.5
            perpendicular_heading = 0
            distance_keypoints = [(0,0,0),
                    (t1, self.centre_airship_distance - 2*self.centre_to_front_bumper, 0)]
            self.gear_mech_on = t1/self.dt # Segments left when gear mech enabled
            self.heading_trajectory = [[0,0,0]]*int(t1/self.dt)
        else:
            t1 = 1 #s, time for segment 1
            rotate_time = self.rotate_arc_length/self.rotate_linear_velocity
            t3 = 1 #s, time for segment 3
            distance_keypoints = [(0,0,0),
                    (t1, self.side_drive_forward_distance-self.delta_s,self.rotate_linear_velocity),
                    (t1 + rotate_time, self.side_drive_forward_distance-self.delta_s+self.rotate_arc_length,self.rotate_linear_velocity),
                    (t1 + rotate_time + t3, self.side_drive_forward_distance+self.rotate_arc_length+self.side_to_wall_distance-2*self.delta_s, 0)
                    ]
            print("Delta S %s, drive_forward_distance sub ds %s, rotate_arc_length %s, rotate_tm %s" % (self.delta_s, self.side_drive_forward_distance-self.delta_s, self.rotate_arc_length, rotate_time))
            self.gear_mech_on = int(t3/self.dt) # Segments left when gear mech enabled
            if self.target is Targets.Left:
                perpendicular_heading = -self.side_rotate_angle
            else:
                perpendicular_heading = self.side_rotate_angle
            self.heading_trajectory=[(0,0,0),]*int(t1/self.dt) \
                +[(t*perpendicular_heading/rotate_time, perpendicular_heading/rotate_time, 0)
                        for t in np.arange(0,rotate_time,self.dt)] \
                +[(perpendicular_heading,0,0),] * int (t3/self.dt)

        self.distance_trajectory = []
        cubic = cubic_generator(distance_keypoints)
        for t in np.arange(0, distance_keypoints[-1][0], self.dt):
            self.distance_trajectory.append(cubic(t))
        # print("Distance Traj Len %s, Heading Traj Len %s" % (len(self.distance_trajectory), len(self.heading_trajectory)))
        # print("Distance trajectory", self.distance_trajectory[int(t1/self.dt)])
        # print("Heading trajectory", self.heading_trajectory[int(t1/self.dt)])
        # print("Distance trajectory", self.distance_trajectory[int(t1/self.dt)+1])
        # print("Heading trajectory", self.heading_trajectory[int(t1/self.dt)+1])

    def on_enable(self):
        super().on_enable()
        self.bno055.resetHeading()
        self.profilefollower.stop()
        self.gearalignmentdevice.reset_position()
        self.geardepositiondevice.retract_gear()
        self.geardepositiondevice.lock_gear()
        self.generate_trajectories()
        self.sd.putBoolean("log", True)

    @state(first=True)
    def drive_to_airship(self, initial_call):
        self.sd.putNumber("left_speed_error", self.chassis.drive_motor_a.getClosedLoopError())
        self.sd.putNumber("right_speed_error", self.chassis.drive_motor_c.getClosedLoopError())
        # Drive to a range where we can close the loop using vision, lidar and
        # gyro to close the loop on position
        if initial_call:
            self.vision.vision_mode = True
            self.profilefollower.modify_queue(heading=self.heading_trajectory,
                    linear=self.distance_trajectory)
            self.profilefollower.execute_queue()
        if len(self.profilefollower.linear_queue) >= self.gear_mech_on:
            self.manipulategear.engage()
        elif not self.profilefollower.executing:
            self.next_state("deploying_gear")

    @state
    def deploying_gear(self, initial_call):
        if initial_call:
            self.manipulategear.engage(initial_state="forward_closed", force=True)
        elif self.manipulategear.current_state == "forward_open":
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

    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    chassis = Chassis

    def __init__(self):
        super().__init__(Targets.Right)

class CentrePegCurves(PegAutonomous):
    MODE_NAME = "Centre Peg Curves"

    manipulategear = ManipulateGear
    profilefollower = ProfileFollower
    chassis = Chassis

    def __init__(self):
        super().__init__(Targets.Centre)
