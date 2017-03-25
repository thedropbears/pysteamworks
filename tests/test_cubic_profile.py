from utilities.profilegenerator import *

def assertAlmostEqual(a, b, tol=1e-3):
    for (x,y) in zip(a,b):
        assert abs(x-y) < tol

def test_zero_velocity_start():
    traj = cubic_generator([(0,0,0), (10, 10, 0)])
    assertAlmostEqual(traj(0)[:2], (0,0))
    assertAlmostEqual(traj(10)[:2], (10,0))

def test_cruise():
    traj = cubic_generator([(0,0,0),
        (5,5,1),
        (10,10,1),
        (15,15,0)])
    assertAlmostEqual(traj(7.5)[:2], (7.5,1))
