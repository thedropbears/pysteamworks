from unittest.mock import MagicMock
import math
from magicbot.magic_tunable import setup_tunables
from automations.filters import VisionFilter
from components.vision import Vision
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

def test_noisy_vision(control, hal_data):

    steps = 150


    vf = VisionFilter()
    vf.vision = MagicMock()
    vf.bno055 = MagicMock()

    vf.vision.x = 0
    vf.vision.time = time.time()
    vf.vision.dt = 1/100
    vf.bno055.getHeading = MagicMock(return_value=0)
    vf.bno055.getHeadingRate = MagicMock(return_value=1)
    vf.vision.horizontal_fov = 61 * math.pi/180

    vf.vision.num_targets=2

    vf.reset()

    measure_tm = 0

    def _on_step(tm, step):
        if step <= 150:
            tm = 3*step/steps
            vision = math.sin(tm)
            if step % 2 == 0:
                vf.vision.time = tm
                vf.last_vision_time = tm-2/50
                vf.last_vision_local_time = time.time()-2/50
                measurement_delay = vf.control_loop_average_delay + vf.vision.dt + 0/50
                measure_tm = tm - measurement_delay
                vf.vision.time = measure_tm

                vf.last_vision_time = tm-2/50-measurement_delay
                vf.last_vision_local_time = time.time()-2/50
                vf.vision.x = np.random.normal(math.sin(measure_tm), math.sqrt(vf.vision_x_variance))
            vf.execute()
            sigma = math.sqrt(vf.filter.P[0][0])
            print("%s,%s,%s,%s,%s,%s" % (tm, math.sin(tm), vf.x, vf.x-3*sigma, vf.x+3*sigma, vf.vision.x))
            assert(vf.x-5*sigma<=vision<=vf.x+5*sigma)
        else:
            return False
        return True

    c = StepController(control, _on_step)
    control.run_test(c)
