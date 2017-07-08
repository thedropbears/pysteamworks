"""This is the code to read and write from the BNO055 IMU that we use.
An IMU, or Inertial Measurement Unit, is a device that takes data from
3 accelerometers and 3 gyros (for the 3 different axis) and converts it
into your orientation in 3 dimensions. We use this in order to get our
heading on the field, which is useful for *field orienting* the robot
as well as holding our heading."""

import enum
import math

import hal
from wpilib import GyroBase, I2C

class BNO055(GyroBase):
    """Class to read euler values in radians from the I2C bus"""

    # i2c addresses
    # a is when the com3 in input is low
    # b is when it is high
    ADDRESS_A = 0X28
    ADDRESS_B = 0X29

    class Address(enum.IntEnum):
        """Register addresses"""

        PAGE_ID = 0X07

        CHIP_ID = 0X00
        ACCEL_REV_ID = 0X01
        MAG_REV_ID = 0X02
        GYRO_REV_ID = 0X03
        SW_REV_ID_LSB = 0X04
        SW_REV_ID_MSB = 0X05
        BL_REV_ID = 0X06

        ACCEL_DATA_X_LSB = 0X08
        ACCEL_DATA_X_MSB = 0X09
        ACCEL_DATA_Y_LSB = 0X0A
        ACCEL_DATA_Y_MSB = 0X0B
        ACCEL_DATA_Z_LSB = 0X0C
        ACCEL_DATA_Z_MSB = 0X0D

        MAG_DATA_X_LSB = 0X0E
        MAG_DATA_X_MSB = 0X0F
        MAG_DATA_Y_LSB = 0X10
        MAG_DATA_Y_MSB = 0X11
        MAG_DATA_Z_LSB = 0X12
        MAG_DATA_Z_MSB = 0X13

        GYRO_DATA_X_LSB = 0X14
        GYRO_DATA_X_MSB = 0X15
        GYRO_DATA_Y_LSB = 0X16
        GYRO_DATA_Y_MSB = 0X17
        GYRO_DATA_Z_LSB = 0X18
        GYRO_DATA_Z_MSB = 0X19

        EULER_H_LSB = 0X1A  # euler heading, least significant bit
        EULER_H_MSB = 0X1B  # euler heading, most significant bit
        EULER_P_LSB = 0X1C  # euler pitch, least significant bit
        EULER_P_MSB = 0X1D  # euler pitch, most significant bit
        EULER_R_LSB = 0X1E  # euler roll, least significant bit
        EULER_R_MSB = 0X1F  # euler roll, most significant bit

        QUATERNION_DATA_W_LSB = 0X20
        QUATERNION_DATA_W_MSB = 0X21
        QUATERNION_DATA_X_LSB = 0X22
        QUATERNION_DATA_X_MSB = 0X23
        QUATERNION_DATA_Y_LSB = 0X24
        QUATERNION_DATA_Y_MSB = 0X25
        QUATERNION_DATA_Z_LSB = 0X26
        QUATERNION_DATA_Z_MSB = 0X27

        LINEAR_ACCEL_DATA_X_LSB = 0X28
        LINEAR_ACCEL_DATA_X_MSB = 0X29
        LINEAR_ACCEL_DATA_Y_LSB = 0X2A
        LINEAR_ACCEL_DATA_Y_MSB = 0X2B
        LINEAR_ACCEL_DATA_Z_LSB = 0X2C
        LINEAR_ACCEL_DATA_Z_MSB = 0X2D

        GRAVITY_DATA_X_LSB = 0X2E
        GRAVITY_DATA_X_MSB = 0X2F
        GRAVITY_DATA_Y_LSB = 0X30
        GRAVITY_DATA_Y_MSB = 0X31
        GRAVITY_DATA_Z_LSB = 0X32
        GRAVITY_DATA_Z_MSB = 0X33

        TEMP = 0X34

        CALIB_STAT = 0X35
        SELFTEST_RESULT = 0X36
        INTR_STAT = 0X37

        SYS_CLK_STAT = 0X38
        SYS_STAT = 0X39
        SYS_ERR = 0X3A

        UNIT_SEL = 0X3B  # address used to select the unit outputs

        DATA_SELECT = 0X3C

        OPR_MODE = 0X3D  # the address used to select what sensors to use for the gyro outputs

        PWR_MODE = 0X3E

        SYS_TRIGGER = 0X3F
        TEMP_SOURCE = 0X40

        AXIS_MAP_CONFIG = 0X41
        AXIS_MAP_SIGN = 0X42

        SIC_MATRIX_0_LSB = 0X43
        SIC_MATRIX_0_MSB = 0X44
        SIC_MATRIX_1_LSB = 0X45
        SIC_MATRIX_1_MSB = 0X46
        SIC_MATRIX_2_LSB = 0X47
        SIC_MATRIX_2_MSB = 0X48
        SIC_MATRIX_3_LSB = 0X49
        SIC_MATRIX_3_MSB = 0X4A
        SIC_MATRIX_4_LSB = 0X4B
        SIC_MATRIX_4_MSB = 0X4C
        SIC_MATRIX_5_LSB = 0X4D
        SIC_MATRIX_5_MSB = 0X4E
        SIC_MATRIX_6_LSB = 0X4F
        SIC_MATRIX_6_MSB = 0X50
        SIC_MATRIX_7_LSB = 0X51
        SIC_MATRIX_7_MSB = 0X52
        SIC_MATRIX_8_LSB = 0X53
        SIC_MATRIX_8_MSB = 0X54

        ACCEL_OFFSET_X_LSB = 0X55
        ACCEL_OFFSET_X_MSB = 0X56
        ACCEL_OFFSET_Y_LSB = 0X57
        ACCEL_OFFSET_Y_MSB = 0X58
        ACCEL_OFFSET_Z_LSB = 0X59
        ACCEL_OFFSET_Z_MSB = 0X5A

        MAG_OFFSET_X_LSB = 0X5B
        MAG_OFFSET_X_MSB = 0X5C
        MAG_OFFSET_Y_LSB = 0X5D
        MAG_OFFSET_Y_MSB = 0X5E
        MAG_OFFSET_Z_LSB = 0X5F
        MAG_OFFSET_Z_MSB = 0X60

        GYRO_OFFSET_X_LSB = 0X61
        GYRO_OFFSET_X_MSB = 0X62
        GYRO_OFFSET_Y_LSB = 0X63
        GYRO_OFFSET_Y_MSB = 0X64
        GYRO_OFFSET_Z_LSB = 0X65
        GYRO_OFFSET_Z_MSB = 0X66

        ACCEL_RADIUS_LSB = 0X67
        ACCEL_RADIUS_MSB = 0X68
        MAG_RADIUS_LSB = 0X69
        MAG_RADIUS_MSB = 0X6A

    class OperationMode(enum.IntEnum):
        """operation modes, changes what sensors it uses and how it fuses them"""
        CONFIG = 0X00
        ACCONLY = 0X01
        MAGONLY = 0X02
        GYRONLY = 0X03
        ACCMAG = 0X04
        ACCGYRO = 0X05
        MAGGYRO = 0X06
        AMG = 0X07
        IMUPLUS = 0X08
        COMPASS = 0X09
        M4G = 0X0A
        NDOF_FMC_OFF = 0X0B
        NDOF = 0X0C

    class PowerMode(enum.IntEnum):
        NORMAL = 0X00
        LOWPOWER = 0X01
        SUSPEND = 0X02

    class AxisMapConfigMode(enum.IntEnum):
        # axis map configuration modes
        P0 = 0X21
        P1 = 0X22
        P2 = 0X23
        P3 = 0X24
        P4 = 0X25
        P5 = 0X26
        P6 = 0X27
        P7 = 0X28

    class AxisMapSign(enum.IntEnum):
        P0 = 0X04
        P1 = 0X00
        P2 = 0X06
        P3 = 0X02
        P4 = 0X03
        P5 = 0X01
        P6 = 0X07
        P7 = 0X05

    # the units that we want and the index in the unit select register that corrosponds to it
    BNO055_UNIT_SEL_ACC_UNIT = 0  # m/s
    BNO055_UNIT_SEL_ACC_UNIT_INDEX = 0
    BNO055_UNIT_SEL_GYR_UNIT = 1  # rad/s
    BNO055_UNIT_SEL_GYR_UNIT_INDEX = 1
    BNO055_UNIT_SEL_EUL_UNIT = 1  # rad
    BNO055_UNIT_SEL_EUL_UNIT_INDEX = 2
    BNO055_UNIT_SEL_TEMP_UNIT = 0  # celcius
    BNO055_UNIT_SEL_TEMP_UNIT_INDEX = 4
    BNO055_UNIT_SEL_ORI_UNIT = 1  # android orientation mode, pitch turning clockwise decreases values
    BNO055_UNIT_SEL_ORI_UNIT_INDEX = 7
    BNO055_UNIT_SEL_LIST = [
        [BNO055_UNIT_SEL_ACC_UNIT, BNO055_UNIT_SEL_ACC_UNIT_INDEX],
        [BNO055_UNIT_SEL_GYR_UNIT, BNO055_UNIT_SEL_GYR_UNIT_INDEX],
        [BNO055_UNIT_SEL_EUL_UNIT, BNO055_UNIT_SEL_EUL_UNIT_INDEX],
        [BNO055_UNIT_SEL_TEMP_UNIT, BNO055_UNIT_SEL_TEMP_UNIT_INDEX],
        [BNO055_UNIT_SEL_ORI_UNIT, BNO055_UNIT_SEL_ORI_UNIT_INDEX]
    ]

    def __init__(self, port=None, address=None):
        super().__init__()
        if address is None:
            address = self.ADDRESS_A
        if port is None:
            port = I2C.Port.kMXP

        sim_port = None
        if hal.isSimulation():
            from .bno055_sim import BNO055Sim
            sim_port = BNO055Sim()

        self.i2c = I2C(port, address, sim_port)

        # set the units that we want
        self.offset = 0.0
        current_units = self.i2c.read(self.Address.UNIT_SEL, 1)[0]
        for wanted, index in self.BNO055_UNIT_SEL_LIST:
            if wanted == 1:
                current_units = current_units | (1 << index)
            elif wanted == 0:
                current_units = current_units & ~(1 << index)
        self.i2c.write(self.Address.UNIT_SEL, current_units)
        self.setOperationMode(self.OperationMode.IMUPLUS)  # accelerometer and gyro
        self.reverse_axis(False, False, False)

    def reverse_axis(self, x, y, z):
        """Reverse the axis directions, xyz are booleans"""
        current_directions = self.i2c.read(self.Address.AXIS_MAP_SIGN, 1)[0]
        if x:
            current_directions = current_directions | (1 << 2)
        else:
            current_directions = current_directions & ~(1 << 2)
        if y:
            current_directions = current_directions | (1 << 1)
        else:
            current_directions = current_directions & ~(1 << 1)
        if z:
            current_directions = current_directions | (1 << 0)
        else:
            current_directions = current_directions & ~(1 << 0)
        self.i2c.write(self.Address.AXIS_MAP_SIGN, current_directions)

    def setOperationMode(self, mode):
        if not 0X00 <= mode <= 0X0C:
            raise ValueError("invalid operation mode %s" % mode)
        self.i2c.write(self.Address.OPR_MODE, mode)

    def getAngle(self):
        """Function called by the GyroBase's PID Source to get the
        current measurement"""
        return self.getHeading()

    def getAngles(self):
        """ Return the [heading, pitch, roll] of the gyro """
        return [self.getHeading(), self.getPitch(), self.getRoll()]

    def getHeading(self):
        angle = (self.getRawHeading() - self.offset)
        return math.atan2(math.sin(angle), math.cos(angle))

    def getRawHeading(self):
        return -self.getEuler(self.Address.EULER_H_LSB)

    def getPitch(self):
        return self.getEuler(self.Address.EULER_P_LSB)

    def getRoll(self):
        return self.getEuler(self.Address.EULER_R_LSB)

    def getEuler(self, start_register):
        euler_bytes = self.i2c.read(start_register, 2)
        euler_int = int.from_bytes(euler_bytes, 'little', signed=True)
        return euler_int / 900.0

    def getHeadingRate(self):
        return -self.getEuler(self.Address.GYRO_DATA_Z_LSB)

    def resetHeading(self, heading=0):
        self.offset = self.getRawHeading() - heading

    def execute(self):
        pass  # Keep MagicBot happy!
