import numpy as np
from magicbot.state_machine import AutonomousStateMachine, state

from automations.profilefollower import ProfileFollower
from components.bno055 import BNO055
from components.gears import GearDepositor
from utilities.profilegenerator import cubic_generator


class DriveForwardAutonomous(AutonomousStateMachine):
    MODE_NAME = "Drive Forward Auto"
    DEFAULT = True

    # injectables
    bno055 = BNO055
    gear_depositor = GearDepositor
    profilefollower = ProfileFollower

    centre_to_front_bumper = 0.49
    centre_airship_distance = 2.93
    dt = 0.02

    def generate_trajectories(self):
        t1 = 1.5
        distance_keypoints = [
            (0, 0, 0),
            (t1, self.centre_airship_distance - 2*self.centre_to_front_bumper, 0)
        ]
        cubic = cubic_generator(distance_keypoints)
        self.distance_trajectory = [cubic(t) for t in np.arange(0, distance_keypoints[-1][0], self.dt)]

    def on_enable(self):
        super().on_enable()
        self.bno055.resetHeading()
        self.profilefollower.stop()
        self.gear_depositor.retract_gear()
        self.gear_depositor.lock_gear()
        self.generate_trajectories()

    @state(first=True)
    def drive_to_airship(self, initial_call):
        # Drive to a range where we can close the loop using vision, lidar and
        # gyro to close the loop on position
        if initial_call:
            self.profilefollower.modify_queue(heading=0, linear=self.distance_trajectory)
            self.profilefollower.execute_queue()
        if not self.profilefollower.executing:
            self.done()
