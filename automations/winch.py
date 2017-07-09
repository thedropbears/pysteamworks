from magicbot import StateMachine, state
from networktables import NetworkTable

from components.winch import Winch


class WinchAutomation(StateMachine):
    # Injectables
    sd = NetworkTable
    winch = Winch

    @state(first=True, must_finish=True)
    def catch_rope(self, state_tm):
        self.put_dashboard()
        self.winch.open_piston()
        self.winch.rotate(0.3)
        self.winch.disable_compressor()
        if self.winch.is_rope_engaged() and state_tm > 2:
            self.next_state("fire_piston")

    @state(must_finish=True)
    def fire_piston(self, state_tm):
        self.put_dashboard()
        self.winch.close_piston()
        self.winch.rotate(0.0)
        self.winch.disable_compressor()
        if state_tm > 0.2:
            self.next_state("climb")

    @state(must_finish=True)
    def climb(self):
        self.put_dashboard()
        self.winch.close_piston()
        self.winch.rotate(1.0)

    def put_dashboard(self):
        """Update all the variables on the smart dashboard"""
        self.sd.putString("state", "climbing")
