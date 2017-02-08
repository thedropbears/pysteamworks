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
        if not self.vision.x > 0.05 or not self.vision.x < -0.05:
            self.gearalignmentdevice.align(self.vision.x)
        else:
            self.gearalignmentdevice.stopMotors()
            self.next_state("endstate")

    @state
    def endstate(self):
        pass

'''    def inrange(self):
        if lidar < 1: #unknown
            self.next_state("usePistons")

    @state    
    def usePistons(self):
        geardepositiondevice.placeGear()
        time.sleep(3)
        geardepositiondevice.reversePiston()

    def put_dashboard(self):
        """Update all the variables on the smart dashboard"""
        pass'''
