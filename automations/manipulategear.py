from magicbot import StateMachine, state, timed_state
from components.geardepositiondevice import GearDepositionDevice
from components.gearalignmentdevice import GearAlignmentDevice
from networktables import NetworkTable
from components.vision import Vision
from components.range_finder import RangeFinder

class ManipulateGear(StateMachine):
    gearalignmentdevice = GearAlignmentDevice
    geardepositiondevice = GearDepositionDevice
    range_finder = RangeFinder
    sd = NetworkTable
    aligned = False
    vision = Vision

    @state(first=True, must_finish=True)
    def align_peg(self):
        # do something to align with the peg
        # now move to the next state
        #move forward
        self.sd.putString("state", "unloadingGear")
        self.put_dashboard()
        if -0.1 <= self.vision.x <= 0.1:
            self.gearalignmentdevice.stop_motors()
            aligned = True
            if self.range_finder.getDistance() < 0.5:
                self.next_state_now("push_gear")
        elif -0.3 <= self.vision.x <= 0.3:
            if self.vision.x > 0.1:
                self.gearalignmentdevice.align(0.5)
            if self.vision.x < 0.1:
                self.gearalignmentdevice.align(-0.5)
            aligned = False
        else:
            if self.vision.x > 0.1:
                self.gearalignmentdevice.align(1)
            if self.vision.x < 0.1:
                self.gearalignmentdevice.align(-1)
            aligned = False

    @timed_state(duration=0.5, next_state="drop_gear", must_finish=True)
    def push_gear(self):
        self.put_dashboard()
        self.geardepositiondevice.push_gear()

    @timed_state(duration=2.0, next_state="retract_gear", must_finish=True)
    def drop_gear(self):
        self.put_dashboard()
        self.geardepositiondevice.drop_gear()

    @timed_state(duration=0.5, next_state="lock_gear", must_finish=True)
    def retract_gear(self):
        self.put_dashboard()
        self.geardepositiondevice.drop_gear()

    @state
    def lock_gear(self):
        self.put_dashboard()
        self.geardepositiondevice.lock_gear()

        self.sd.putString("state", "stationary")
        self.done()

    def put_dashboard(self):
        """Update all the variables on the smart dashboard"""
        self.sd.putNumber("vision_x", self.vision.x)
        self.sd.putNumber("smoothed_vision_x", self.vision.smoothed_x)
        self.sd.putNumber("vision_y", self.range_finder.getDistance())
