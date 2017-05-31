from components.range_finder import RangeFinder
from components.chassis import Chassis
from components.vision import Vision
from utilities.kalman import Kalman
from components.bno055 import BNO055

import time

import math

import numpy as np

class RangeFilter:

    chassis = Chassis
    range_finder = RangeFinder
    bno055 = BNO055
    vision = Vision

    # the range sensor noise
    range_variance = 0.0005

    # the encoder noise
    odometry_variance = 1e-3

    loop_dt = 1/50

    reset_thresh = 0.5

    vision_based_range_variance = 0.1

    def __init__(self):
        pass

    def setup(self):
        self.reset()
        self.last_vision_mode = self.vision.enabled

    def reset(self):

        self.is_reset = True

        r = self.range_finder.getDistance()

        # initial state
        x_hat = np.array([r]).reshape(-1, 1)

        # initial covariance is range variance as we initialise filter with a
        # measurement
        P = np.array([[self.range_variance]])

        # process noise is just error in odometry prediction
        Q = np.array([self.odometry_variance])

        # measurement noise is LIDAR error
        R = np.array([[self.range_variance]])

        # initialise the filter
        self.filter = Kalman(x_hat, P, Q, R)

        self.last_odometry = np.array(self.chassis.get_raw_wheel_distances())


    def predict(self):
        F = np.identity(1)

        odometry = np.array(self.chassis.get_raw_wheel_distances())

        # calculate the movement of the robot since the last timestep
        robot_center_movement = -np.mean(odometry-self.last_odometry)
        # robot center movement is control input
        u = np.array([[robot_center_movement]])
        # control input model is identity
        B = np.identity(1)

        self.filter.predict(F, u, B)

        self.last_odometry = odometry

    def range_update(self):
        r = self.range_finder.getDistance()

        if not abs(r - self.range) < math.sqrt(self.filter.P[0,0])*5:
            return
        elif abs(r - self.range) > self.reset_thresh:
            self.reset()
            return

        # measurement is range
        z = np.array([[r]])
        H = np.identity(1)

        self.filter.update(z, H)

    def execute(self):
        # we only want to run the range filter when we are tracking the vision
        # targets on the tower, as that is the only situation where we need
        # range data. Store a variable so we only reset once each time the
        # vision is disabled.
        if not self.vision.enabled:
            if self.last_vision_mode:
                self.reset()
            self.last_vision_mode = self.vision.enabled
            return
        self.last_vision_mode = self.vision.enabled

        # range state should always be within this range. if it isnt, reset
        if self.range < 0.1 or self.range >= 5:
            r = self.range_finder.getDistance()
            if r is 40:
                self.logger.info("WARNING: range filter being reset")
                self.reset()
        self.predict()
        self.range_update()

    @property
    def range(self):
        return self.filter.x_hat[0,0]

    def vision_predicted_range(self):
        """ Predict what our range finder reading should be based off of the
        distance between the vision targets. Used in order to gate the ranges
        that are used to update our filter. """
        return self.vision.derive_target_range()
