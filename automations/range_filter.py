from components.range_finder import RangeFinder
from components.chassis import Chassis
from components.vision import Vision
from utilities.kalman import Kalman
from components.bno055 import BNO055

import time

import math

import numpy as np

from collections import deque

class RangeFilter:

    chassis = Chassis
    range_finder = RangeFinder
    bno055 = BNO055
    vision = Vision

    state_vector_size = 1

    # the range sensor noise
    range_variance = 0.0005

    # the encoder noise
    odometry_variance = 1e-5

    loop_dt = 1/50

    reset_thresh = 3

    vision_based_range_variance = 0.05

    def __init__(self):
        pass

    def setup(self):
        self.reset()

    def reset(self):

        r = self.range_finder.getDistance()
        if float(r) is float('inf'):
            r = 0

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

        self.range_deque = deque(maxlen=50)
        self.odometry_deque = deque(maxlen=50)
        self.update_deques()

        self.last_vision_time = self.vision.time

    def update_deques(self):
        self.odometry_deque.append(np.array(self.chassis.get_raw_wheel_distances()))
        r = self.range_finder.getDistance()
        self.range_deque.append(40 if float(r) is float('inf') else r)

    def predict(self, timesteps=1):
        F = np.identity(self.state_vector_size)
        B = np.identity(self.state_vector_size)

        robot_center_movement = -np.mean(self.odometry_deque[-timesteps]-self.odometry_deque[-timesteps-1])

        u = np.array([robot_center_movement]).reshape(-1, 1)

        self.filter.predict(F, u, B)

        self.last_odometry = np.array(self.chassis.get_raw_wheel_distances())

    def range_update(self, timesteps=1):
        r = self.range_deque[-timesteps]

        if not abs(r - self.range) < math.sqrt(self.filter.P[0][0])*5:
            return
        elif abs(r - self.range) > self.reset_thresh:
            self.reset()
            return

        z = np.array([r]).reshape(-1, 1)
        H = np.identity(self.state_vector_size)

        self.filter.update(z, H)

    def vision_update(self):
        z = np.array([self.vision_predicted_range()]).reshape(-1, 1)
        H = np.identity(self.state_vector_size)
        R = np.array([self.vision_based_range_variance]).reshape(-1, 1)

        self.filter.update(z, H, R)

    def execute(self):
        self.update_deques()
        if self.range < 0.1 or self.range >= 40:
            r = self.range_deque[-1]
            if float(r) is float('inf'):
                print("reset")
                self.reset()
        timesteps_since_vision = int((time.time() - self.vision.time)/50)
        self.predict()
        self.range_update()
        if self.vision.time != self.last_vision_time and self.vision_predicted_range() != 0:
            self.filter.roll_back(timesteps_since_vision)
            self.vision_update()
            for i in range(timesteps_since_vision):
                self.predict(timestep=timesteps_since_vision-i)
                self.range_update(timestep=timesteps_since_vision-i)
            self.last_vision_time = self.vision.time

    @property
    def range(self):
        return self.filter.x_hat[0][0]

    def vision_predicted_range(self):
        """ Predict what our range finder reading should be based off of the distance between
        the vision targets. Used in order to gate the ranges that are used to update our kalman
        filter. """
        if self.vision.target_sep == 0:
            return 0
        target_angle = 0
        if self.bno055.getHeading() > math.pi/6:
            # we think that we are looking at the right hand target
            target_angle = math.pi/3
        elif self.bno055.getHeading() < -math.pi/6:
            # we think that we are looking at the left hand target
            target_angle = -math.pi/3
        perpendicular_to_heading = target_angle - self.bno055.getHeading()
        theta_alpha = math.pi/2 - perpendicular_to_heading - self.vision.derive_vision_angle()
        predicted_range = self.vision.derive_target_range() * math.sin(perpendicular_to_heading * math.pi/2) / math.sin(theta_alpha)
        return predicted_range
