# this is the chassis class, a good example of a component. it is a
# bit special in that it has to take direct numerical input from the
# joystick (which is achieved by changing the 'input' variable, which
# is a list composed of [vx, vy, vz, throttle]) but is a good example
# of what a component should look like.

from ctre import CANTalon

class Chassis:
    
    TALON_NUMBERS = [0, 1, 2, 3]

    def __init__(self):
        super().__init__()
        self.sticks = [0, 0]
        self.motors = [CANTalon(num) for num in self.TALON_NUMBERS]
        self.motors[1].setControlMode(CANTalon.ControlMode.Follower)
        self.motors[1].set(self.TALON_NUMBERS[0])
        self.motors[3].setControlMode(CANTalon.ControlMode.Follower)
        self.motors[3].set(self.TALON_NUMBERS[2])

    def setup(self):
        """Run just after createObjects.
        Useful if you want to run something after just once after the
        robot code is started, that depends on injected variables"""

    def on_enable(self):
        """Run every time the robot transitions to being enabled"""
        pass

    def on_disable(self):
        """Run every time the robot transitions to being disabled"""
        pass

    def execute(self):
        """Run at the end of every control loop iteration"""
        self.motors[0].set(self.sticks[0])
        self.motors[2].set(self.sticks[1])
