from magicbot import StateMachine, state, timed_state
from components.geardepositiondevice import GearDepositionDevice
from components.gearalignmentdevice import GearAlignmentDevice
from networktables import NetworkTable
from components.vision import Vision
from components.range_finder import RangeFinder
from automations.profilefollower import ProfileFollower

class ManipulateGear(StateMachine):
    gearalignmentdevice = GearAlignmentDevice
    geardepositiondevice = GearDepositionDevice
    range_finder = RangeFinder
    profilefollower = ProfileFollower
    sd = NetworkTable
    aligned = False
    vision = Vision

    place_gear_range = 0.6
    align_tolerance = 0.05

    @state(first=True)
    def init(self):
        self.geardepositiondevice.retract_gear()
        self.geardepositiondevice.lock_gear()
        self.gearalignmentdevice.reset_position()
        self.next_state("align_peg")


    @state(must_finish=True)
    def align_peg(self):
        # do something to align with the peg
        # now move to the next state
        #move forward
        self.put_dashboard()
        # print("align peg, vision %s" % (self.vision.x))

        if (-self.align_tolerance <= self.vision.x <= self.align_tolerance
                and not self.profilefollower.queue[0]):
            self.gearalignmentdevice.stop_motors()
            aligned = True
            if self.range_finder.getDistance() < self.place_gear_range:
                self.next_state_now("forward_closed")
        else:
            # print("align_vision")
            self.gearalignmentdevice.align()
            aligned = False

    @timed_state(duration=0.5, next_state="forward_open", must_finish=True)
    def forward_closed(self):
        self.put_dashboard()
        self.geardepositiondevice.push_gear()

    @timed_state(duration=0.25, must_finish=True)
    def forward_open(self):
        self.put_dashboard()
        self.geardepositiondevice.drop_gear()

    def put_dashboard(self):
        """Update all the variables on the smart dashboard"""
        self.sd.putString("state", "unloadingGear")
