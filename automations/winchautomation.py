import components.winch
from magicbot import StateMachine, state
from components import GearAligmentDevice, GearDepositionDevice
from networktables import NetworkTable

class AutonomousWinch(StateMachine):

    def driver_press(self):
        
        self.engage()

    @state(first=True)
    def rope_caught(self):
        current = winch_motor.getOutputCurrent()
        for current > 5:
            for current > 5:
                rope_lock_solenoid
                self.next_state("touchpad_pressed")

    @state
    def touchpad_pressed


    def put_dashboard(self):
        """Update all the variables on the smart dashboard"""
        pass
