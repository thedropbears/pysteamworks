from magicbot import StateMachine, state
from components.geardepositiondevice import GearDepositionDevice
from components.gearalignmentdevice import GearAlignmentDevice
from networktables import NetworkTable
from components.vision import Vision
import time

class ManipulateGear(StateMachine):
    gearalignmentdevice = GearAlignmentDevice
    geardepositiondevice = GearDepositionDevice
    sd = NetworkTable
    aligned = False
    vision = Vision
  #  lidar = int

    # example first state
    @state(first=True)
    def peg_align(self):
        # do something to align with the peg
        # now move to the next state
        #move forward      
        if -0.1 <= self.vision.x <= 0.1:
            self.gearalignmentdevicestopMotors()
            aligned = True
            self.next_state("usePistons")
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

    @state
    def usePistons(self):
        self.geardepositiondevice.push_gear()
        self.geardepositiondevice.drop_gear()
        # Wait 3 seconds (add in a timed_state), then
        self.geardepositiondevice.retract_gear()
        self.geardepositiondevice.lock_gear()

    def put_dashboard(self):
        """Update all the variables on the smart dashboard"""
        pass
