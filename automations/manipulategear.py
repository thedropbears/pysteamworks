from magicbot import StateMachine, state
from components.geardepositiondevice import GearDepositionDevice
from components.gearalignmentdevice import GearAlignmentDevice
from networktables import NetworkTable

class ManipulateGear(StateMachine):
    gearalignmentdevice = GearAlignmentDevice
    geardepositiondevice = GearDepositionDevice
    sd = NetworkTable

    # example first state
    @state(first=True)
    def peg_align(self):
        # do something to align with the peg
        # now move to the next state
        self.next_state("deposit_gear")

    @state
    def deposit_gear(self):
        pass

    def put_dashboard(self):
        """Update all the variables on the smart dashboard"""
        pass
