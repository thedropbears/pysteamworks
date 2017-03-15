from magicbot import StateMachine, state, timed_state
from components.geardepositiondevice import GearDepositionDevice
from components.gearalignmentdevice import GearAlignmentDevice
from networktables import NetworkTable
from components.vision import Vision
from components.range_finder import RangeFinder
from components.chassis import Chassis
from automations.profilefollower import ProfileFollower

class ManipulateGear(StateMachine):
    gearalignmentdevice = GearAlignmentDevice
    geardepositiondevice = GearDepositionDevice
    range_finder = RangeFinder
    chassis = Chassis
    profilefollower = ProfileFollower
    sd = NetworkTable
    aligned = False
    vision = Vision

    place_gear_range = 0.5
    align_tolerance = 0.05

    deploy_jitter = 0.1

    move_back_close_tol = 0.2

    push_gear_input_tolerance = 0.05

    @state(first=True)
    def init(self):
        self.vision.vision_mode = True
        self.geardepositiondevice.retract_gear()
        self.geardepositiondevice.lock_gear()
        self.gearalignmentdevice.reset_position()
        self.gearalignmentdevice.position_mode()
        self.next_state("align_peg")


    @state(must_finish=True)
    def align_peg(self):
        # do something to align with the peg
        # now move to the next state
        #move forward
        self.put_dashboard()
        self.vision.vision_mode = True

        if (-self.align_tolerance <= self.vision.x <= self.align_tolerance
                and not self.profilefollower.queue[0]
                and abs(self.chassis.inputs[0]) < self.push_gear_input_tolerance
                and abs(self.chassis.inputs[2]) < self.push_gear_input_tolerance):
            self.gearalignmentdevice.stop_motors()
            aligned = True
            r = self.range_finder.getDistance()
            if r<0.1:
                r = 40
            if r < self.place_gear_range:
                self.chassis.input_enabled = False
                if self.gearalignmentdevice.get_rail_pos() >= 0:
                    self.gearalignmentdevice.set_position(self.gearalignmentdevice.get_rail_pos()-self.deploy_jitter)
                elif self.gearalignmentdevice.get_rail_pos() < 0:
                    self.gearalignmentdevice.set_position(self.gearalignmentdevice.get_rail_pos()+self.deploy_jitter)
                self.next_state_now("forward_closed")
        else:
            self.gearalignmentdevice.align()
            aligned = False

    @state(must_finish=True)
    def forward_closed(self, state_tm):
        self.put_dashboard()
        self.geardepositiondevice.push_gear()
        if state_tm > 0.5:
            self.next_state("forward_open")

    @state(must_finish=True)
    def forward_open(self, state_tm):
        self.put_dashboard()
        self.geardepositiondevice.drop_gear()
        if state_tm > 0.5:
            self.next_state("backward_open")

    @state(must_finish=True)
    def backward_open(self, initial_call):
        self.put_dashboard()
        self.geardepositiondevice.retract_gear()
        self.chassis.input_enabled = True
        if initial_call:
            self.initial_distances = self.chassis.get_raw_wheel_distances()
        if ((abs(abs(self.initial_distances[0]) - abs(self.chassis.get_raw_wheel_distances()[0]))
            +abs(abs(self.initial_distances[1]) - abs(self.chassis.get_raw_wheel_distances()[1])))
            / 2 > self.move_back_close_tol):
            self.next_state_now("backward_close")

    @state(must_finish=True)
    def backward_close(self):
        self.put_dashboard()
        self.geardepositiondevice.lock_gear()
        self.done()

    def done(self):
        super().done()
        self.vision.vision_mode = False

    def put_dashboard(self):
        """Update all the variables on the smart dashboard"""
        self.sd.putString("state", "unloadingGear")
