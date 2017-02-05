from magicbot import StateMachine, state
from components.geardepositiondevice import GearDepositionDevice
from components.gearalignmentdevice import GearAlignmentDevice
from networktables import NetworkTable
from components.vision import Vision

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
        if not vision.x > 0.2 or not vision.x < -0.2:
            GearAlignmentDevice.align(vision)
        else:
            GearAlignmentDevice.stopMotors()
            self.next_state("deposit_gear")

    @state
    def deposit_gear(self):
        pass
        '''
        if lidar < 1: #unknown
            geardepositiondevice.placeGear()'''

    def put_dashboard(self):
        """Update all the variables on the smart dashboard"""
        pass
