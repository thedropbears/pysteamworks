import numpy as np
from collections import deque
from itertools import islice

def m_dot(*args):
    ret = args[0]
    for a in args[1:]:
        ret = np.dot(ret,a)
    return ret

class Kalman:

    def __init__(self, x_hat, P, Q, R=None, f=None, h=None,
            residual_x=None, residual_z=None, history_len=50):
        """
        :param x_hat: the initial state of the system
        :param P: the covariance matrix of the initial state of the system
        :param Q: the covariance matrix that represents additional
        uncertainty added each prediction step (this implementation
        assumes that Q is constant)
        :param f: the state transition function applied between each time
        for nonlinear systems. Takes the form:
            f(x_k-1, u) -> x_k
        where x and k are numpy arrays of shape (n, 1)
        :param h: the observation function applied where the observation
        model is nonlinear. Takes the form:
            h(x) -> z
        where x and z are numpy arrays of shape (n, 1)

        :param residual_x: for non subtraction supported systems, funciton to calculate
        difference between one state and another. (e.g. when calculating
        difference between angle of 0 and 2pi radians)
        :param residual_z: for non subtraction supported systems, funciton to calculate
        difference between one measurement and another. (e.g. when calculating
        difference between angle of 0 and 2pi radians)
        """
        self.x_hat = x_hat
        self.P = P
        self.Q = Q
        self.R = R
        self.history=deque(iterable=[[self.x_hat, self.P]], maxlen=history_len)
        self.f = f
        self.h = h

        def subtraction_supported_residual(a,b):
            return a-b
        self.residual_x = residual_x or subtraction_supported_residual
        self.residual_z = residual_z or subtraction_supported_residual

    def roll_back(self, timesteps):
        if timesteps <= 0:
            if timesteps < 0:
                print("WARNING: timesteps to roll back is less than 0")
            return
        self.history = deque(islice(self.history, 0, len(self.history)-timesteps+1))
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

    def update(self, z, H, R=None):
        """
        :param z: the sensor readings in this timestep
        :param H: the matrix that translates x_hat (the state vector)
        into the sensor space (z)
        :param R: the covariance matrix representing the noise in the
        sensor reading. If not supplied, defaults to member variable R.
        """

        R = self.R if R is None else R
        if R is None:
            raise Exception("Error: Kalman filter cannot predict - sensor covariance not supplied.")

        # the difference between the means of the estimated state (x_hat)
        # and the state given by the sensors (z)
        y = z - np.dot(H, self.x_hat)

        # find the normalised product of the two multivariate gaussian
        # distributions, represented by the current state of the system
        # and the sensor readings in this state
        kalman_gain = m_dot(self.P, np.transpose(H), np.linalg.inv(
            m_dot(H, self.P, H.T)+R))

        self.x_hat = self.x_hat + np.dot(kalman_gain, y)

        self.P = (np.identity(len(self.x_hat)) - kalman_gain.dot(H)).dot(self.P)

    def unscented_predict(self, u=None, f=None, Q=None):
        """
        :param u: known control inputs if required for f.
        :param f: state transition function (see constructor). Defaults to one
        provided in constructor. f: R
        :param Q: covariance matrix of the process noise, defaults to
        provided in constructor.
        """

        f = f or self.f
        if f is None:
            raise Exception("Error: Kalman unscented_predict requires a state transition function")

        Q = self.Q if Q is None else Q
        if Q is None:
            raise Exception("Error: Kalman unscented_predict requires a process noise covariance matrix")

        self.x_hat, self.P = Kalman.unscented_transform(self.x_hat, self.P, f=f, u=u, noise_cov=Q)

    def unscented_update(self, z, z_dim=None, R=None, h=None):
        """
        :param z: the measurement
        :param z_dim: the dimension of the measurement vector, f, and of the
        vector returned from the observation model, h. Defaults to dim(z)
        :param h: the observation model function. Defaults to one provided
        in constructor.
        :param R: the noise around the measurement. Defaults to one provided
        in constructor.
        """

        h = h or self.h
        if h is None:
            raise Exception("Error: Kalman unscented_update requires an observation model")

        R = self.R if R is None else R
        if R is None:
            raise Exception("Error: Kalman filter cannot predict - sensor covariance not supplied.")

        z_dim = z_dim or z.size

        # propogate the augmented state through the observation model
        z_hat, P_z_hat, w_c, X, Z = Kalman.unscented_transform(self.x_hat, self.P, f=h, f_n=z_dim, noise_cov=R, update=True)

        # the state-measurement cross-covariance matrix
        P_xz = np.zeros(shape=(self.x_hat.size, z_hat.size))
        for i in range(2*self.x_hat.size+1):
            P_xz += (w_c[i]
                    * np.dot(self.residual_x(X[i].reshape(-1,1), self.x_hat),
                            self.residual_z(Z[i].reshape(-1,1), z_hat).T))

        # compute the kalman gain
        K = np.dot(P_xz, np.linalg.inv(P_z_hat))

        # compute the innovation
        y = self.residual_z(z, z_hat)

        self.x_hat = self.x_hat + K.dot(y)

        self.P = self.P - m_dot(K, P_z_hat, K.T)

    @staticmethod
    def unscented_transform(x, P_x, f, f_n=None, u=None, noise_cov=None, alpha=1e-3, beta=2, kappa=0, update=False):
        """
        :param x: the mean of the distribution to be transformed
        :param P_x: the covariance matrix of the distribution to be transformed
        :param f: the vector-valued function to be applied to the distribution. Must
        accept a numpy array of the same dimension as x.
        :prama f_n: the dimension of the column vector returned from f. Defaults to dim(x)
        """
        if f_n is None:
            f_n = x.size

        n = x.size
        lmda =  alpha**2*(n+kappa)-n

        # sigma vectors stored on rows
        sigma_points = np.zeros(shape=(2*n+1, n))
        sigma_points[0] = x.reshape(n)

        # use the cholesky decomposition to compute
        # matrix square root
        sigma_offsets = np.linalg.cholesky((n+lmda)*P_x)

        for i in range(1, 2*n+1):
            addition = sigma_offsets[:,i-1] if i<=n else -sigma_offsets[:,i-n-1]
            sigma_points[i] = x.reshape(n) + addition

        w_m = np.zeros(shape=(2*n+1))
        w_m[0] = lmda/(n+lmda)

        w_c = np.zeros(shape=(2*n+1))
        w_c[0] = w_m[0] + (1 - alpha**2 + beta)

        for i in range(1, 2*n+1):
            w = 1/(2*(n+lmda))
            w_m[i] = w
            w_c[i] = w

        Y = np.zeros(shape=(2*n+1, f_n))

        y = np.zeros(shape=(f_n, 1))
        P_y = np.zeros(shape=(f_n, f_n))

        for i in range(2*n+1):
            if u is not None:
                Y[i] = f(sigma_points[i].reshape(-1,1), u).reshape(f_n)
            else:
                Y[i] = f(sigma_points[i].reshape(-1,1)).reshape(f_n)
            y += w_m[i]*Y[i].reshape(-1,1)

        for i in range(2*n+1):
            Y_i = Y[i].reshape(-1, 1)
            P_y += w_c[i]*np.dot(Y_i-y, (Y_i-y).T)

        if noise_cov is not None:
            P_y += noise_cov

        if update:
            return y, P_y, w_c, sigma_points, Y

        return y, P_y
