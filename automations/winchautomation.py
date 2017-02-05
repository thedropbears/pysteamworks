import components.winch
from magicbot import StateMachine, state
from components import GearAligmentDevice, GearDepositionDevice
from networktables import NetworkTable
from ctre import CANTalon


class AutonomousWinch(StateMachine):
    self.engage()

    def driver_press(self):
        current = winch_motor.getOutputCurrent()

    @state(first=True)
    def retractPiston(self):
        rope_lock_solenoid.set(wpilib.DoubleSolenoid.KReverse)

    @timed_state(duration=1)
    def onMotor(self):
        winch_motor_spin()
        self.next_state("ropeCaught")

    @state
    def retractPiston(self):
        rope_lock_solenoid_reverse()
        self.next_state("rotateWinch")

    @state
    def rotateWinch(selfl):
        current = winch_motor.getOutputCurrent()
        if current > 5:
            rope_lock_solenoid_forward()
            self.next_state("firePiston")
        else:
            rotateWinch

    @state
    def firePiston(self):
        if reset button pressed:
            self.next_state("retractPiston")
        else:
            self.next_state("touchpadPressed")

    @state
    def touchpadPressed(self):
        if touchpadPressed:
            self.next_state("stopMotor")
        else:
            rotateWinch()

    @state
    def stopMotor(self):
        finished

    def putDashboard(self):
        """Update all the variables on the smart dashboard"""
        pass
