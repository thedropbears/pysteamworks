from components.winch import Winch
from magicbot import StateMachine, state, timed_state
from networktables import NetworkTable


class AutonomousWinch(StateMachine):
    winch = Winch

    @state(first=True)
    def retractPiston(self):
        self.winch.rope_lock_solenoid_reverse()
        self.next_state("onMotor")

    @timed_state(duration=1, must_finish=True, next_state="rotateWinch")
    def onMotor(self):
        self.winch.rotate_winch(0.08)

    @state(must_finish=True)
    def rotateWinch(self):
        if self.winch.on_rope_engaged():
            self.next_state("firePiston")
        else:
            self.winch.rotate_winch(0.08)

    @state(must_finish=True)
    def firePiston(self):
        self.winch.rope_lock_solenoid_forward()
        self.winch.rotate_winch(1)
        self.next_state("onMotorFull")

    @timed_state(duration=1, must_finish=True, next_state="touchpadPressed")
    def onMotorFull(self):
        pass

    @state(must_finish=True)
    def touchpadPressed(self):
        if self.winch.on_touchpad_engaged():
            self.next_state("stopMotor")

    @state(must_finish=True)
    def stopMotor(self):
        self.winch.rotate_winch(0)
        self.done()

    def putDashboard(self):
        """Update all the variables on the smart dashboard"""
        pass
