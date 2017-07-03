from components.vision import Vision
from utilities.kalman import Kalman

from collections import deque

import numpy as np
import time

from magicbot import MagicRobot

class VisionFilter:

    vision = Vision

    state_vector_size = 2

    # the initial uncertainty in the vision rate
    init_dx_variance = 0.5

    # the vision sensor noise
    vision_x_variance = 0.0005

    init_x_variance = 0.001

    # the variance in the unknown acceleration impulse
    acceleration_variance = 2

    loop_dt = 1/50

    reset_thresh = 0.2

    control_loop_average_delay = MagicRobot.control_loop_wait_time/2

    def __init__(self):
        # create the state transition matrix
        self.F = np.identity(self.state_vector_size)
        self.F[0,1] = self.loop_dt

    def setup(self):
        self.reset()

    def reset(self):

        timesteps_since_vision = int((time.time() - self.vision.time)/50)

        # if there have only been several timesteps since the last vision
        # measurement, the last measurement is likely to be a better estimate
        # of the true state of the system than 0
        start_x = 0
        if timesteps_since_vision < 10:
            start_x = self.vision.x

        # starting state
        x_hat = np.array([start_x, 0]).reshape(-1, 1)

        # set our starting variances
        P = np.zeros(shape=(self.state_vector_size, self.state_vector_size))
        P[0,0] = VisionFilter.init_x_variance
        P[1,1] = VisionFilter.init_dx_variance

        # use newtons equations to figure out the process noise matrix.
        # Process noise modelled as random walk in discrete time.
        G = np.array([[(1/2)*self.loop_dt**2, self.loop_dt]])
        Q = np.dot(G, G.T) * self.acceleration_variance

        self.last_vision = self.vision.x
        self.last_vision_time = self.vision.time

        # create vision noise array
        R = np.array([[self.vision_x_variance]])

        # initialise the filter
        self.filter = Kalman(x_hat, P, Q, R)

        self.last_vision_local_time = time.time()

    def on_enable(self):
        self.reset()

    def predict(self, timestep=1):
        """Predict what the measurement should be in the next timestep.
        :param timestep: the number of timesteps in the past that we are predicting forward *from*"""

        # we have no control inputs, so just make it 0
        B = np.identity(self.state_vector_size)
        u = np.zeros(shape=(self.state_vector_size, 1))

        self.filter.predict(self.F, u, B)

    def update(self):

        # observation model - just the x part of the state. filter then
        # updates the dx part of the state to the extent that x and dx co-vary
        H = np.array([[1, 0]])
        z = np.array([self.vision.x])

        self.filter.update(z, H)

        self.last_vision = self.vision.x
        self.last_vision_time = self.vision.time

    def execute(self):
        # if we havent got a vision measurement, dont do any predict
        # or updating
        if self.vision.time == 0:
            return
        # estimate the delay since the last frame was captured
        vision_delay = self.vision.dt + self.control_loop_average_delay
        # calculate the timesteps since we got the vision measurement
        timesteps_since_vision = int(vision_delay*1/self.loop_dt)
        # if we have waited many timesteps since last frame, dont predict
        if timesteps_since_vision > 10:
            return
        # if filter has diverged or we havent got a framen a while, reset
        elif abs(self.vision.x - self.filter.x_hat[0,0]) > self.reset_thresh:
            self.reset()
        # let the filter predict forward
        self.predict()
        if self.vision.time != self.last_vision_time:
            # if we have a new vision measurement, calculate the time since
            # we got it, then roll back the filter by that much time. then,
            # update with the measurement and re-predict forwardw
            self.last_vision_local_time = time.time()
            to_roll_back = min(timesteps_since_vision, len(self.filter.history))
            self.filter.roll_back(to_roll_back)
            self.update()
            for i in range(timesteps_since_vision):
                self.predict(timestep=timesteps_since_vision-i)

    @property
    def x(self):
        return self.filter.x_hat[0][0]

    @property
    def dx(self):
        return self.filter.x_hat[1][0]

    @property
    def angle(self):
        """Give the linearised angle to vision target based on the
        horizontal field of view"""
        return -(self.x*self.horizontal_fov/2)
