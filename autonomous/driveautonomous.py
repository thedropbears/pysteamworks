import numpy as np

from automations.profilefollower import ProfileFollower
from magicbot.state_machine import AutonomousStateMachine, state
from networktables import NetworkTable

from components.geardepositiondevice import GearDepositionDevice
from utilities.profilegenerator import cubic_generator
from components.chassis import Chassis
from components.bno055 import BNO055
from components.vision import Vision

class DriveForwardAutonomous(AutonomousStateMachine):
    MODE_NAME = "Drive Forward Auto"
    DEFAULT=True

    bno055 = BNO055
    vision = Vision
    geardepositiondevice = GearDepositionDevice
    profilefollower = ProfileFollower
    centre_to_front_bumper = 0.49
    centre_airship_distance = 2.93
    dt = 0.02

    def __init__(self):
        super().__init__()

    def generate_trajectories(self):
        t1 = 1.5
        perpendicular_heading = 0
        distance_keypoints = [(0, 0, 0),
            (t1, self.centre_airship_distance - 2*self.centre_to_front_bumper, 0)]
        self.gear_mech_on = t1/self.dt # Segments left when gear mech enabled
        self.heading_trajectory = [(0, 0, 0)] * int(t1/self.dt)
        self.distance_trajectory = []
        cubic = cubic_generator(distance_keypoints)
        for t in np.arange(0, distance_keypoints[-1][0], self.dt):
            self.distance_trajectory.append(cubic(t))

    def on_enable(self):
        super().on_enable()
        self.bno055.resetHeading()
        self.profilefollower.stop()
        self.geardepositiondevice.retract_gear()
        self.geardepositiondevice.lock_gear()
        self.generate_trajectories()

    @state(first=True)
    def drive_to_airship(self, initial_call):
        # Drive to a range where we can close the loop using vision, lidar and
        # gyro to close the loop on position
        if initial_call:
            self.profilefollower.modify_queue(heading=self.heading_trajectory,
                    linear=self.distance_trajectory)
            self.profilefollower.execute_queue()
        if not self.profilefollower.executing:
            self.done()

    def done(self):
        super().done()
