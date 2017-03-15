import numpy as np
from collections import deque
from itertools import islice

def m_dot(*args):
    ret = args[0]
    for a in args[1:]:
        ret = np.dot(ret,a)
    return ret

class Kalman:

    def __init__(self, x_hat, P, Q, R, history_len=50):
        """
        :param x_hat: the initial state of the system
        :param P: the covariance matrix of the initial state of the system
        :param Q: the covariance matrix that represents additional
        uncertainty added each prediction step (this implementation
        assumes that Q is constant)
        """
        self.x_hat = x_hat
        self.P = P
        self.Q = Q
        self.R = R
        self.history=deque(iterable=[[self.x_hat, self.P]], maxlen=history_len)

    def roll_back(self, timesteps):
        if timesteps == 0:
            return
        self.history = deque(islice(self.history, 0, -timesteps))
        self.x_hat = self.history[-1][0]
        self.P = self.history[-1][1]

    def predict(self, F, u, B):
        """
        :param u_k: the control inputs (or other known influences on the
        system state)
        :param B: the matrix that applies the control inputs to the state
        vector
        :param F: the state prediciton matrix for this timestep
        """
        # evolve the state according to the last state, control inputs, and
        # prodiction/control matrices
        # self.x_hat = m_dot(F, self.x_hat) + m_dot(u, B)
        self.x_hat = m_dot(F, self.x_hat) + B.dot(u)

        # evolve the state's covariance matrix (uncertainty surronding the
        # state) by scaling by the prodiction matrix and adding the unknown
        # process noise
        self.P = m_dot(F, self.P, np.transpose(F)) + self.Q
        self.history.append([self.x_hat, self.P])

    def update(self, z, H):
        """
        :param z: the sensor readings in this timestep
        :param H: the matrix that translates x_hat (the state vector)
        into the sensor space (z)
        :param R: the covariance matrix representing the noise in the
        sensor reading
        """
        # the difference between the means of the estimated state (x_hat)
        # and the state given by the sensors (z)
        y = z - np.dot(H, self.x_hat)

        # find the normalised product of the two multivariate gaussian
        # distributions, represented by the current state of the system
        # and the sensor readings in this state
        kalman_gain = m_dot(self.P, np.transpose(H), np.linalg.inv(
            m_dot(H, self.P, H.T)+self.R))

        self.x_hat = self.x_hat + np.dot(kalman_gain, y)

        # the new covariance matrix around the new mean
        self.P = self.P - m_dot(kalman_gain, H, self.P)
