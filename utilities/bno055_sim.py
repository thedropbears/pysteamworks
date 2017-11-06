"""Contains code that lets us test the BNO code"""

import math
import struct

from hal_impl.i2c_helpers import I2CSimBase
from .bno055 import BNO055

class BNO055Sim(I2CSimBase):
    heading = math.pi / 2.0
    pitch = -math.pi / 8.0
    roll = 0.01

    def transactionI2C(self, port, device_address, data_to_send, send_size, data_received, receive_size):
        '''
            To give data back use ``data_received``::

                data_received[:] = [1,2,3...]

            :returns: number of bytes returned
        '''

        if data_to_send[0] == BNO055.Address.EULER_H_LSB:
            struct.pack_into('<h', data_received, 0, int(self.heading * 900.0))
        if data_to_send[0] == BNO055.Address.EULER_P_LSB:
            struct.pack_into('<h', data_received, 0, int(self.pitch * 900.0))
        if data_to_send[0] == BNO055.Address.EULER_R_LSB:
            struct.pack_into('<h', data_received, 0, int(self.roll * 900.0))

        return receive_size
