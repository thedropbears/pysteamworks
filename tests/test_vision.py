import numpy as np

import vision


def test_black():
    res = vision.find_target(np.zeros(shape=(320, 240, 3), dtype=np.uint8))
    assert res[2] is 0
