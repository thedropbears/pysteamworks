from utilities.bno055 import BNO055
from utilities.bno055_sim import BNO055Sim

epsilon = 0.01

def test_euler_angles():
    bno055 = BNO055()
    heading, pitch, roll = bno055.getAngles()
    assert abs(heading - -BNO055Sim.heading) < epsilon #heading direction reversed in bno class
    assert abs(pitch - BNO055Sim.pitch) < epsilon
    assert abs(roll - BNO055Sim.roll) < epsilon

def test_reset_heading():
    bno055 = BNO055()
    heading = bno055.getHeading()
    assert heading != 0.0
    bno055.resetHeading(0.0)
    heading = bno055.getHeading()
    assert heading == 0.0
    bno055.resetHeading(2.0)
    heading = bno055.getHeading()
    assert heading == 2.0
