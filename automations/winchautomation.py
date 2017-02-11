from components.winch import Winch
from magicbot import StateMachine, state, timed_state
from networktables import NetworkTable
from ctre import CANTalon


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
            self.winch.rope_lock_solenoid_forward()
            self.winch.rotate_winch(1)
            self.next_state("firePiston")
        else:
            self.winch.rotate_winch(0.08)
            print(self.winch.winch_motor.getOutputCurrent())

    @state
    def firePiston(self):
        pass
#        if reset button pressed:
#            self.next_state("retractPiston")
#        else:
#            self.next_state("touchpadPressed")

    @state
    def touchpadPressed(self):
        pass
#        if touchpadPressed:
#            self.next_state("stopMotor")
#        else:
#            rotateWinch()

    @state
    def stopMotor(self):
        pass
 #       finished

    def putDashboard(self):
        """Update all the variables on the smart dashboard"""
        pass
