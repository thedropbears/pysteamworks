from magicbot.state_machine import AutonomousStateMachine, state
from automations.manipulategear import ManipulateGear

class PegAutonomous(AutonomousStateMachine):

    manipulategear = ManipulateGear

    class Targets:
        Left = 0
        Middle = 1
        Right = 2

    def __init__(self, target=Targets.Middle):
        pass

    @state(first=True)
    def drive_to_airship(self):
        # Drive to a range where we can close the loop using vision, lidar and
        # gyro to close the loop on position

        #drive to the airship, then
        self.next_state("drive_to_wall")

    @state
    def drive_to_wall(self):
        # Drive from the range that vision and the lidar can detect the wall
        # to the wall itself
        pass
