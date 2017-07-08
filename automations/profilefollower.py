import math
from collections import deque
from networktables import NetworkTable

from components.bno055 import BNO055
from components.chassis import Chassis


class ProfileFollower:
    # injectables
    bno055 = BNO055
    chassis = Chassis
    sd = NetworkTable

    # linear motion feedforward/back gains
    kP = 6 # proportional gain
    kV = 1 # feedforward gain
    kI = 0.3 # integral gain
    kD = 1 # derivative gain
    kA = 0.0 # acceleration feedforward gain

    # heading motion feedforward/back gains
    kPh = 6 # proportional gain
    kVh = 1 # feedforward gain
    kIh = 0.2 # integral gain
    kDh = 40 # derivative gain

    def __init__(self):
        # queues of segments to be executed
        self.linear_queue = deque()
        self.heading_queue = deque()
        self.executing = False

    def modify_queue(self, heading, linear=None, overwrite=False):
        """Modify the motion profiling queue, or overwrite it."""
        if isinstance(heading, (int, float)):
            heading = [(heading, 0.0, 0.0)] * len(linear)
        if not linear:
            linear = [(0.0, 0.0, 0.0)] * len(heading)
        if overwrite:
            self.linear_queue = deque(linear)
            self.heading_queue = deque(heading)
        else:
            self.linear_queue.extend(linear)
            self.heading_queue.extend(heading)

    def execute_queue(self):
        """Start executing the queue that is in the motion profile buffer.

        Also resets encoder positions in the chassis, and disables driver
        input to the chassis in teleop.
        """
        # ensure that there is a queue to execute from
        if self.linear_queue:
            self.executing = True
            self.chassis.input_enabled = False
            self.chassis.set_enc_pos()
            # clear integators
            self.position_error_i = 0
            self.heading_error_i = 0
            self.last_position_error = 0
            self.last_heading_error = 0
        else:
            self.executing = False

    def stop(self):
        """Stop executing the motion profile.

        Also clears the MP queue, and re-enables chassis driver input.
        """
        self.executing = False
        self.linear_queue.clear()
        self.heading_queue.clear()
        self.chassis.input_enabled = True
        # clear integators
        self.heading_error_i = 0
        self.position_error_i = 0

    def execute(self):
        if self.executing:
            # get the next linear and angular segments from the front of the
            # queue
            linear_seg = self.linear_queue.popleft()
            heading_seg = self.heading_queue.popleft()

            # get the current left and right positions of the wheels
            left_pos, right_pos = self.chassis.get_wheel_distances()

            pos = (left_pos + right_pos) / 2 # average the two wheel distances
            # calculate the position errror
            pos_error = linear_seg[0] - pos
            # calucate the derivative of the position error
            self.d_pos_error = (pos_error - self.last_position_error)
            # sum the position error over the timestep
            self.position_error_i += pos_error

            # generate the linear output to the chassis (m/s)
            linear_output = (self.kP*pos_error + self.kV*linear_seg[1]
                + self.kA*linear_seg[2] + self.kI*self.position_error_i
                + self.kD*self.d_pos_error)

            self.sd.putNumber("distance_error_mp", pos_error)
            self.sd.putNumber("linear_output_mp", linear_output)

            # get the current heading of the robot since last reset
            heading = self.bno055.getRawHeading() - self.bno055.offset
            # calculate the heading error
            heading_error = heading_seg[0] - heading
            # wrap heading error, stops jumping by 2 pi from the gyro
            heading_error = math.atan2(
                math.sin(heading_error),
                math.cos(heading_error)
            )
            # sum the heading error over the timestep
            self.heading_error_i += heading_error
            # calculate the derivative of the heading error
            d_heading_error = (heading_error - self.last_heading_error)

            # generate the rotational output to the chassis
            heading_output = (
                self.kPh*heading_error + self.kVh*heading_seg[1]
                + self.heading_error_i*self.kIh + d_heading_error*self.kDh)

            # store the current errors to be used to compute the
            # derivatives in the next timestep
            self.last_heading_error = heading_error
            self.last_position_error = pos_error

            self.sd.putNumber("heading_error_mp", heading_error)

            if self.linear_queue:
                self.chassis.set_velocity(linear_output, heading_output)
            else:
                self.chassis.set_velocity(linear_seg[1], heading_seg[1])

        if not self.linear_queue:
            self.executing = False
