from magicbot import StateMachine, state
from networktables import NetworkTable

from automations.filters import RangeFilter, VisionFilter
from automations.profilefollower import ProfileFollower
from components.bno055 import BNO055
from components.chassis import Chassis
from components.gears import GearAlignmentDevice, GearDepositionDevice
from components.range_finder import RangeFinder
from components.vision import Vision
from utilities.profilegenerator import generate_trapezoidal_trajectory


class ManipulateGear(StateMachine):
    # Injectables
    bno055 = BNO055
    chassis = Chassis
    gearalignmentdevice = GearAlignmentDevice
    geardepositiondevice = GearDepositionDevice
    profilefollower = ProfileFollower
    range_filter = RangeFilter
    range_finder = RangeFinder
    sd = NetworkTable
    vision = Vision
    vision_filter = VisionFilter

    place_gear_range = 0.32
    align_tolerance = 0.02

    deploy_jitter = 0.1

    move_back_close_tol = 0.3

    push_gear_input_tolerance = 0.05

    rail_travel = 0.4

    @state(first=True)
    def init(self):
        self.vision.enabled = True
        self.geardepositiondevice.retract_gear()
        self.geardepositiondevice.lock_gear()
        self.gearalignmentdevice.reset_position()
        self.vision_filter.reset()
        self.range_filter.reset()
        self.next_state("align_peg")

    @state(must_finish=True)
    def align_peg(self):
        # do something to align with the peg
        # now move to the next state
        #move forward
        self.put_dashboard()

        if (-self.align_tolerance <= self.vision_filter.x <= self.align_tolerance
                and not self.profilefollower.executing
                and abs(self.chassis.inputs[0]) < self.push_gear_input_tolerance
                and abs(self.chassis.inputs[2]) < self.push_gear_input_tolerance):
            # r = self.range_finder.getDistance()
            r = self.range_filter.range
            if r < 0.1:
                r = 40
            if r < self.place_gear_range:
                pass
                # self.chassis.input_enabled = False
                # if self.gearalignmentdevice.get_rail_pos() >= 0:
                #     self.gearalignmentdevice.set_position(self.gearalignmentdevice.get_rail_pos()-self.deploy_jitter)
                # elif self.gearalignmentdevice.get_rail_pos() < 0:
                #     self.gearalignmentdevice.set_position(self.gearalignmentdevice.get_rail_pos()+self.deploy_jitter)
                # self.next_state_now("forward_closed")
        elif self.vision.num_targets > 1:
            self.gearalignmentdevice.align()

    @state(must_finish=True)
    def forward_closed(self, state_tm):
        self.put_dashboard()
        self.geardepositiondevice.push_gear()
        if state_tm > 0.5:
            self.next_state("forward_open")

    @state(must_finish=True)
    def forward_open(self, initial_call, state_tm):
        self.put_dashboard()
        self.geardepositiondevice.drop_gear()
        self.chassis.input_enabled = True
        if initial_call:
            self.profilefollower.stop()
            roll_back = generate_trapezoidal_trajectory(
                    0, 0, -0.4, 0, 3,
                    2, -3, 50)
            self.profilefollower.modify_queue(self.bno055.getHeading(),
                    linear=roll_back, overwrite=True)
            self.profilefollower.execute_queue()
        if initial_call:
            self.initial_distances = self.chassis.get_raw_wheel_distances()
        if ((abs(abs(self.initial_distances[0]) - abs(self.chassis.get_raw_wheel_distances()[0]))
            + abs(abs(self.initial_distances[1]) - abs(self.chassis.get_raw_wheel_distances()[1])))
                / 2) > self.move_back_close_tol:
            self.next_state_now("backward_open")

    @state(must_finish=True)
    def backward_open(self, initial_call, state_tm):
        self.put_dashboard()
        self.geardepositiondevice.retract_gear()
        if not self.profilefollower.executing:
            self.next_state("backward_close")

    @state(must_finish=True)
    def backward_close(self):
        self.put_dashboard()
        self.geardepositiondevice.lock_gear()
        self.done()

    def done(self):
        super().done()
        self.vision.enabled = False

    def put_dashboard(self):
        """Update all the variables on the smart dashboard"""
        self.sd.putString("state", "unloadingGear")
