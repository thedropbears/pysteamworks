from components.winch import Winch
from magicbot import StateMachine, state, timed_state
from networktables import NetworkTable


class WinchAutomation(StateMachine):
    winch = Winch
    sd = NetworkTable

    @state(first=True, must_finish=True)
    def catch_rope(self, state_tm):
        self.put_dashboard()
        self.winch.piston_open()
        self.winch.rotate_winch(0.3)
        if self.winch.on_rope_engaged() and state_tm > 2:
            self.next_state("climb")

    @state(must_finish=True)
    def climb(self):
        self.put_dashboard()
        self.winch.piston_close()
        self.winch.rotate_winch(1)

    #    if self.winch.on_touchpad_engaged() and state_tm > 2:
    #        self.next_state("press_touchpad")

    # @state(must_finish=True)
    # def press_touchpad(self, state_tm):
    #     self.put_dashboard()
    #     self.winch.rotate_winch(0)
    #     self.sd.putString("state", "stationary")
    #     self.done()

    def put_dashboard(self):
        """Update all the variables on the smart dashboard"""
        self.sd.putString("state", "climbing")
