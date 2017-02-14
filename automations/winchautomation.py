from components.winch import Winch
from magicbot import StateMachine, state, timed_state
from networktables import NetworkTable
 

class WinchAutomation(StateMachine):
    winch = Winch
    sd = NetworkTable

    @state(first=True)
    def retract_piston(self):
        self.sd.putString("state", "climbing")
        self.winch.rope_lock_solenoid_reverse()
        self.next_state("on_motor")

    @timed_state(duration=2, must_finish=True, next_state="rotate_winch")
    def on_motor(self):
        self.winch.rotate_winch(0.3)

    @state(must_finish=True)
    def rotate_winch(self):
        if self.winch.on_rope_engaged():
            self.next_state("fire_piston")

    @state(must_finish=True)
    def fire_piston(self):
        self.winch.rope_lock_solenoid_forward()
        self.winch.rotate_winch(0.8)
        self.next_state("on_motor_full")

    @timed_state(duration=2, must_finish=True, next_state="touchpad_pressed")
    def on_motor_full(self):
        pass

    @state(must_finish=True)
    def touchpad_pressed(self):
        if self.winch.on_touchpad_engaged():
            self.next_state("stop_motor")

    @state(must_finish=True)
    def stop_motor(self):
        self.winch.rotate_winch(0)
        self.sd.putString("state", "stationary")
        self.done()
