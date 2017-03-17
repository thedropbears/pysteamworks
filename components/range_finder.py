import wpilib
from wpilib.interfaces import PIDSource
from networktables import NetworkTable
import hal


class RangeFinder:

    def __init__(self, dio_number=0):
        self.range_finder_counter = wpilib.Counter(dio_number, mode=wpilib.Counter.Mode.kPulseLength)
        self.range_finder_counter.setSemiPeriodMode(highSemiPeriod=True)
        self.range_finder_counter.setSamplesToAverage(10)
        self._smoothed_d = 0.0
        self.sd = NetworkTable.getTable('SmartDashboard')

    def _getDistance(self):
        if not hal.isSimulation():
            # 10 usec is 1cm, return as metres
            return self.range_finder_counter.getPeriod() * 1000000 / 1000
        else:
            return 0.0

    def getDistance(self):
        # TODO: adjust based on calculated heading
        return self._getDistance()

    def execute(self):
        # get the distance and smooth it
        alpha = 0.7
        d = self._getDistance()
        if d > 40:  # Max range is around 40m
            d = 40
        self._smoothed_d = alpha * d + (1.0 - alpha) * self._smoothed_d
