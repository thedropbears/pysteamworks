import math

import wpilib


class RangeFinder:
    counter = wpilib.Counter

    def __init__(self):
        self._smoothed_d = 0.0

    def setup(self):
        self.counter.setSemiPeriodMode(highSemiPeriod=True)
        self.counter.setSamplesToAverage(10)

    def _getDistance(self):
        # 10 usec is 1cm, return as metres
        return self.counter.getPeriod() * 1000000 / 1000

    def getDistance(self):
        r = self._getDistance()
        if math.isinf(r):
            r = 40
        return r

    def execute(self):
        # get the distance and smooth it
        alpha = 0.7
        d = self._getDistance()
        if d > 40:  # Max range is around 40m
            d = 40
        self._smoothed_d = alpha * d + (1.0 - alpha) * self._smoothed_d
