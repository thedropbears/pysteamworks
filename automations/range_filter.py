from components.range_finder import RangeFinder
from components.chassis import Chassis
from utilities.kalman import Kalman
from components.bno055 import BNO055

import numpy as np

from collections import deque

class RangeFilter:

    range_hist_len = 10

    state_vector_size = 1

    # the range sensor noise
    range_variance = 0.00001

    # the encoder noise
    odometry_variance = 1e-6

    loop_dt = 1/50

    chassis = Chassis
    range_finder = RangeFinder

    range_change_update_thresh = 0.1
    range_std_update_thresh = 0.2

    reset_thresh = 3

    bno055 = BNO055

    # scaling factor as uncertainty increases with range
    vision_predicted_range_sf = 0.25

    def __init__(self):
        pass

    def setup(self):
        self.reset()

    def reset(self):

        r = self.range_finder.getDistance()
        if float(r) is float('inf'):
            r = 0

        self.range_deque = deque(iterable=[self.range_finder.getDistance()], maxlen=self.range_hist_len)

        self.is_reset = True

        r = self.range_finder.getDistance()
        if float(r) is float('inf'):
            r = 40

        # starting state
        x_hat = np.array([r]).reshape(-1, 1)

        P = np.zeros(shape=(self.state_vector_size, self.state_vector_size))
        P[0][0] = self.range_variance
        Q = np.array([self.odometry_variance]).reshape(self.state_vector_size, self.state_vector_size)

        # error vision and error rate of change of vision are correlated
        R = np.array([self.range_variance]).reshape(self.state_vector_size, self.state_vector_size)
        self.filter = Kalman(x_hat, P, Q, R)

        self.last_odometry = np.array(self.chassis.get_raw_wheel_distances())

    def predict(self):
        F = np.identity(self.state_vector_size)
        B = np.identity(self.state_vector_size)

        robot_center_movement = -np.mean(np.array(self.chassis.get_raw_wheel_distances())-self.last_odometry)

        u = np.array([robot_center_movement]).reshape(-1, 1)

        self.filter.predict(F, u, B)

        self.last_odometry = np.array(self.chassis.get_raw_wheel_distances())

    def update(self):
        r = self.range_finder.getDistance()
        if float(r) is float('inf'):
            return
        if abs(self.vision_predicted_range - r) > self.vision_predicted_range_sf*self.vision.derive_target_range():
            return
        elif abs(r - self.filter.x_hat[0][0]) > self.reset_thresh:
            self.reset()

        z = np.array([r]).reshape(-1, 1)
        H = np.identity(self.state_vector_size)

        self.filter.update(z, H)

    def execute(self):
        if self.range < 0.1 or self.range == 40:
            r = self.range_finder.getDistance()
            if not float(r) is float('inf'):
                self.reset()
            return
        self.predict()
        self.update()

    @property
    def range(self):
        return self.filter.x_hat[0][0]

    @property
    def vision_predicted_range(self):
        """ Predict what our range finder reading should be based off of the distance between
        the vision targets. Used in order to gate the ranges that are used to update our kalman
        filter. """
        target_angle = 0
        if self.bno055.getHeading() > math.pi/6:
            # we think that we are looking at the right hand target
            target_angle = math.pi/3
        elif self.bno055.getHeading() < -math.pi/6:
            # we think that we are looking at the left hand target
            target_angle = -math.pi/3
        perpendicular_to_heading = target_angle - self.bno055.getHeading()
        theta_alpha = math.pi/2 - perpendicular_to_heading - self.vision.derive_vision_angle
        predicted_range = self.vision.derive_target_range * math.sin(perpendicular_to_heading * math.pi/2) / math.sin(theta_alpha)
        return predicted_range
