from magicbot import StateMachine, state, timed_state
from components.geardepositiondevice import GearDepositionDevice
from components.gearalignmentdevice import GearAlignmentDevice
from networktables import NetworkTable
from components.vision import Vision
from components.range_finder import RangeFinder
import time

class ManipulateGear(StateMachine):
    gearalignmentdevice = GearAlignmentDevice
    geardepositiondevice = GearDepositionDevice
    range_finder = RangeFinder
    sd = NetworkTable
    aligned = False
    vision = Vision
    checked = False
    # example first state
    @state(first=True, must_finish=True)
    def pegAlign(self):
        # do something to align with the peg
        # now move to the next state
        #move forward
        self.put_dashboard()
        if -0.1 <= self.vision.x <= 0.1:
            self.gearalignmentdevice.stopMotors()
            aligned = True
            self.next_state_now("measureDistance")
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

    @state(must_finish=True)
    def measureDistance(self):
        self.put_dashboard()
        if self.range_finder.getDistance() < 0.25:
            if not self.checked:
                self.checked = True
                self.next_state("pegAlign")
            else:
                self.next_state("openPistons")
        else:
            self.next_state("pegAlign")

    @timed_state(duration=3.0, next_state="closePistons", must_finish=True)
    def openPistons(self):
        self.put_dashboard()
        self.geardepositiondevice.push_gear()
        time.sleep(0.1)
        self.geardepositiondevice.drop_gear()

    @state(must_finish=True)
    def closePistons(self):
        self.put_dashboard()
        self.geardepositiondevice.lock_gear()
        time.sleep(0.1)
        self.geardepositiondevice.retract_gear()
        self.done()

    def put_dashboard(self):
        """Update all the variables on the smart dashboard"""
        self.sd.putNumber("vision_x", self.vision.x)
        self.sd.putNumber("smoothed_vision_x", self.vision.smoothed_x)
        self.sd.putNumber("vision_y", self.range_finder.getDistance())
