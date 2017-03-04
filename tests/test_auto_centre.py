"""Tests for a single auto routine, adapted from the robotpy_ext code."""

# TODO write a physics.py and use robotpy_ext.autonomous.selector_tests.*

from networktables.util import ChooserControl

auto_seconds = 15


def test_centrepeg(control, fake_time, robot):
    """Test the CentrePeg auto routine."""

    class AutoTester:
        def __init__(self):
            self.initialized = False
            self.init_time = None
            self.chooser = None

            self.state = 'start'
            self.until = None

        def initialize_chooser(self, tm):
            if self.chooser is None:
                self.chooser = ChooserControl('Autonomous Mode')

            if len(self.chooser.getChoices()) == 0:
                return False

            self.state = 'start'
            self.until = tm
            self.init_time = tm
            self.initialized = True

        def on_step(self, tm):
            if not self.initialized:
                if not self.initialize_chooser(tm):
                    assert tm < 10, "Robot didn't create a chooser within 10 seconds"
                    return True

            if self.state == 'auto':
                if tm >= self.until:
                    self.state = 'finished'
                    control.set_operator_control(enabled=False)
                    return False

            elif self.state == 'start':
                if tm >= self.until:
                    control.set_autonomous()

                    self.state = 'auto'
                    self.until = tm + auto_seconds
                    self.chooser.setSelected('Centre Peg')

            return True

    controller = control.run_test(AutoTester)

    # Make sure it ran for the correct amount of time
    assert int(fake_time.get()) == int(auto_seconds + controller.init_time)
