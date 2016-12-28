# components are used to represent the physical devices on the robot.
# you may have one for the chassis, one for the shooter, or whatever
# else you happen to have on your robot. components are used by the
# main robot code in robot.py to provide high level interfaces to
# low level subsystems.

class GenericComponent:

    def __init__(self):
        super().__init__(self)

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
        pass
