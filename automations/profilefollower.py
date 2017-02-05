from components.chassis import Chassis
from components.bno055 import BNO055
from networktables import NetworkTable

import math

class ProfileFollower:

    # linear motion feedforward/back gains
    #
    # position P controller
    kP = 4
    # velocity and acceleration feedforward
    kV = 1.2
    kA = 0.0

    # heading motion feedforward/back gains
    # heading feedback
    kPh = 4.5
    # angular velocity feedforward
    kVh = 1.2

    chassis = Chassis

    bno055 = BNO055
    sd = NetworkTable

    def __init__(self):
        self.queue = [[],[]]
        self.executing = False

    def modify_queue(self, linear=None, heading=None, overwrite=False):
        if not heading:
            heading = [[0.0, 0.0, 0.0]] * len(linear)
        if not linear:
           linear = [[0.0, 0.0, 0.0]] * len(heading)
        if overwrite:
            self.queue = [linear, heading]
        else:
            self.queue = [self.queue[0]+linear, self.queue[1]+heading]

    def execute_queue(self):
        # ensure that there is a queue to execute from
        if len(self.queue[0]):
            self.executing = True
            self.chassis.input_enabled = False
            self.chassis.set_enc_pos()

    def stop(self):
        self.executing = False
        self.queue = [[], []]
        self.chassis.input_enabled = True

    def execute(self):

        if self.executing:
            linear_seg = self.queue[0].pop(0)
            heading_seg = self.queue[1].pop(0)

            [left_pos, right_pos] = self.chassis.get_wheel_distances()

            pos = (left_pos + right_pos) / 2
            pos_error = linear_seg[0] - pos

            linear_output = (self.kP * pos_error + self.kV * linear_seg[1]
                + self.kA * linear_seg[2])

            self.sd.putNumber("distance_error_mp", pos_error)

            heading = self.bno055.getRawHeading() - self.bno055.offset
            heading_error = heading_seg[0] - heading
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

            heading_output = (
                self.kPh * heading_error + self.kVh * heading_seg[1])

            self.sd.putNumber("heading_error_mp", heading_error)

            self.chassis.set_velocity(linear_output, heading_output)
        if not self.queue[0]:
            self.executing = False
