from utilities.kalman import Kalman
import numpy as np

def test_KF_converge():
    dt=0.1
    F = np.identity(2)
    F[0,1] = dt

    R = np.identity(2)
    H = np.identity(2)

    x = np.array([[1,1]]).T

    x_hat_init = np.array([[0,0]]).T
    # 1 on the diagonals is reasonable starting covariance
    P_init = np.identity(2)

    G = np.array([[dt**2/2, dt]]).T
    Q = np.dot(G, G.T)

    kf = Kalman(x_hat_init, P_init, Q)

    for i in range(100):
        x = np.dot(F, x)

        B = np.zeros(shape=(2, 1))
        u = 0
        kf.predict(F, u, B)

        # we are testing filter covergence, not performance, so give it
        # perfect measurements
        kf.update(x, H, R)

    x_confidence = np.sqrt(kf.P[0,0])*5
    x_dot_confidence = np.sqrt(kf.P[1,1])*5
    assert kf.x_hat[0,0]-x_confidence <= kf.x_hat[0,0] < kf.x_hat[0,0]+x_confidence
    assert kf.x_hat[1,0]-x_dot_confidence <= kf.x_hat[1,0] < kf.x_hat[1,0]+x_dot_confidence

def test_UKF_converge():

    dt=0.1
    F = np.identity(2)
    F[0,1] = dt
    def f(x):
        return np.dot(F, x)

    R = np.identity(2)

    def h(x):
        return x

    def subtraction_supported_residual(a,b):
        return a-b

    x = np.array([[1,1]]).T

    x_hat_init = np.array([[0,0]]).T
    # 1 on the diagonals is reasonable starting covariance
    P_init = np.identity(2)

    G = np.array([[dt**2/2, dt]]).T
    Q = np.dot(G, G.T)

    kf = Kalman(x_hat_init, P_init, Q, f=f, h=h,
            residual_x=subtraction_supported_residual,
            residual_z=subtraction_supported_residual)

    for i in range(100):
        x = np.dot(F, x)

        kf.unscented_predict()

        # we are testing filter covergence, not performance, so give it
        # perfect measurements
        kf.unscented_update(z=x, R=R)

    x_confidence = np.sqrt(kf.P[0,0])*5
    x_dot_confidence = np.sqrt(kf.P[1,1])*5
    assert kf.x_hat[0,0]-x_confidence <= kf.x_hat[0,0] < kf.x_hat[0,0]+x_confidence
    assert kf.x_hat[1,0]-x_dot_confidence <= kf.x_hat[1,0] < kf.x_hat[1,0]+x_dot_confidence
