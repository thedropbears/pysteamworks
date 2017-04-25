from components.vision import Vision
from components.bno055 import BNO055
from utilities.kalman import Kalman

from collections import deque

import numpy as np
import time

from magicbot import MagicRobot

class VisionFilter:

    vision = Vision
    bno055 = BNO055

    state_vector_size = 2

    # the initial uncertainty in the vision rate
    init_dx_variance = 0.01

    # the vision sensor noise
    vision_x_variance = 0.0002

    init_x_variance = 0.001

    # the variance in the unknown acceleration impulse
    acceleration_variance = 0.6

    loop_dt = 1/50

    reset_thresh = 0.2

    control_loop_average_delay = MagicRobot.control_loop_wait_time/2


    def __init__(self):
        pass

    def setup(self):
        self.reset()

    def reset(self):
        self.imu_deque = deque(maxlen=50, iterable=[self.get_heading_state()])

        timesteps_since_vision = int((time.time() - self.vision.time)/50)
        start_x = 0
        if timesteps_since_vision < 10:
            start_x = 0
        # starting state
        x_hat = np.array([start_x, self.imu_deque[0][1]]).reshape(-1, 1)

        P = np.zeros(shape=(self.state_vector_size, self.state_vector_size))
        P[0][0] = VisionFilter.init_x_variance
        P[1][1] = VisionFilter.init_dx_variance
        Q = (np.array(
                [[self.loop_dt**4/4, self.loop_dt**3/3],[self.loop_dt**3/2, self.loop_dt**2]]).
                reshape(self.state_vector_size, self.state_vector_size)*self.acceleration_variance)
        self.last_vision = self.vision.x
        self.last_vision_time = self.vision.time

        R = np.array([[self.vision_x_variance]])

        self.filter = Kalman(x_hat, P, Q, R)

        self.last_vision_local_time = time.time()

    def get_heading_state(self):
        return np.array([0, self.bno055.getHeadingRate()]).reshape(-1, 1)

    def on_enable(self):
        self.reset()

    def predict(self, timestep=1):
        """Predict what the measurement should be in the next timestep.
        :param timestep: the number of timesteps in the past that we are predicting forward *from*"""

        F = np.identity(self.state_vector_size)
        F[0][1] = self.loop_dt
        B = np.identity(self.state_vector_size)

        self.imu_deque.append(self.get_heading_state())

        u = Vision.rad_to_vision_units(self.imu_deque[-timestep] - self.imu_deque[-timestep-1])

        self.filter.predict(F, u, B)

    def update(self):

        x = self.vision.x

        H = np.array([[1, 0]])
        z = np.array([x])

        self.filter.update(z, H)

        self.last_vision = self.vision.x
        self.last_vision_time = self.vision.time

    def execute(self):
        if self.vision.time == 0:
            return
        vision_delay = self.vision.dt + self.control_loop_average_delay
        timesteps_since_vision = int(vision_delay*50)
        if timesteps_since_vision > 10:
            return
        elif abs(self.vision.x - self.filter.x_hat[0][0]) > self.reset_thresh:
            self.reset()
        self.predict()
        if self.vision.time != self.last_vision_time:
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
        return -(self.x*self.horizontal_fov/2)
