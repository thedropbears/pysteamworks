from unittest.mock import MagicMock
import math
from magicbot.magic_tunable import setup_tunables
from automations.range_filter import RangeFilter
import numpy as np
import time

class StepController(object):
    '''
        Robot test controller
    '''

    def __init__(self, control, on_step):
        '''constructor'''
        self.control = control
        self.step = 0
        self._on_step = on_step

    def __call__(self, tm):
        '''Called when a new robot DS packet is received'''
        self.step += 1
        return self._on_step(tm, self.step)

def test_noisy_range_predict(control, hal_data):

    start_range = 3.0
    speed = 1.0 # m/s
    steps = 50*start_range/speed
    odometry_bias = 0.1 # per meter
    vision_max_bias = 0.1 # m

    bad_range_chance = 0.2

    rf = RangeFilter()
    rf.range_finder = MagicMock()
    rf.chassis = MagicMock()
    rf.vision = MagicMock()
    rf.bno055 = MagicMock()

    rf.range_finder.getDistance = MagicMock(return_value=np.random.normal(loc=start_range, scale=math.sqrt(rf.range_variance)))
    rf.chassis.get_raw_wheel_distances = MagicMock(return_value=0.0)

    rf.reset()

    r = start_range

    def _on_step(tm, step):
        r = start_range - (step/steps)*start_range
        if step <= steps:
            rf.range_finder.getDistance = MagicMock(return_value=np.random.normal(loc=r, scale=math.sqrt(rf.range_variance)))
            if np.random.uniform() <= bad_range_chance:
                rf.range_finder.getDistance = MagicMock(return_value=np.random.uniform(0, 1))
            rf.chassis.get_raw_wheel_distances = MagicMock(return_value=[start_range-r+(odometry_bias*(step/steps))]*2)
            vision_bias = vision_max_bias*math.sin(2*math.pi*step/steps)
            rf.vision_predicted_range = MagicMock(return_value=np.random.normal(loc=r+vision_bias, scale=math.sqrt(rf.vision_based_range_variance)))
            rf.vision.time = time.time()
            rf.execute()
            # TODO: make this a test not a simulation :p
            # sigma = math.sqrt(rf.filter.P[0][0])
            # print("%s %s %s %s %s %s" % (r, rf.range, rf.range-3*sigma, rf.range+3*sigma, rf.range_finder.getDistance(), rf.vision_predicted_range()))
        else:
            assert abs(rf.range)<0.02
            return False
        return True

    c = StepController(control, _on_step)
    control.run_test(c)
