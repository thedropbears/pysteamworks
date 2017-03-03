from utilities.profilegenerator import *

def test_zero_velocity_start():
    traj = generate_trapezoidal_trajectory(0, 0, 1, 0, 10, 1, -1)
    assert traj[0][0:2] == (0, 0)
    assert traj[-1][0:2] == (1, 0)
    traj = generate_trapezoidal_trajectory(0, 0, 10, 0, 1, 1, -1)
    assert traj[0][0:2] == (0, 0)
    assert traj[-1][0:2] == (10, 0)

def test_asymmetric_accel():
    traj = generate_trapezoidal_trajectory(0, 0, 1, 0, 10, 2, -1)
    assert traj[0][0:2] == (0, 0)
    assert traj[-1][0:2] == (1, 0)
    traj = generate_trapezoidal_trajectory(0, 0, 10, 0, 1, 2, -1)
    assert traj[0][0:2] == (0, 0)
    assert traj[-1][0:2] == (10, 0)

def test_non_zero_velocity_start():
    traj = generate_trapezoidal_trajectory(0, 1, 1, 0, 10, 2, -1)
    assert traj[0][1] == 1
    assert traj[-1][0:2] == (1, 0)
    traj = generate_trapezoidal_trajectory(0, 1, 10, 0, 1, 2, -1)
    assert traj[0][1] == 1
    assert traj[-1][0:2] == (10, 0)

