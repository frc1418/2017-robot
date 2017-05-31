import hal
import wpilib

from magicbot import state, timed_state, StateMachine, tunable
from networktables import NetworkTable
from networktables.util import ntproperty

from components.swervedrive import SwerveDrive
from controllers.pos_controller import XPosController, YPosController
from controllers.angle_controller import AngleController
from controllers.position_history import PositionHistory


class AutoAlign(StateMachine):
    drive = SwerveDrive

    x_ctrl = XPosController
    y_ctrl = YPosController
    angle_ctrl = AngleController

    pos_history = PositionHistory

    cv_enabled = ntproperty('/camera/control/cv_enabled', False)

    ideal_skew = tunable(-0.967)
    ideal_angle = tunable(-1.804)

    def __init__(self):
        target = None

        nt = NetworkTable.getTable('/camera')
        nt.addTableListener(self._on_target, True, 'target')

        self.aimed_at_angle = None
        self.aimed_at_x = None

    def _on_target(self, source, key, value, isNew):
        self.target = value

    def _move_to_position(self):
        target = self.target

        if target is not None and len(target) > 0:
            target = target[:]

            angle, skew, capture_ts = target
            history = self.pos_history.get_position(capture_ts)

            if history is not None:
                r_angle, r_x, r_y, r_time = history

                self.aimed_at_angle = r_angle + angle - self.ideal_angle

            self.target = None

        if self.aimed_at_angle is not None:
            self.angle_ctrl.align_to(self.aimed_at_angle)

        return self.angle_ctrl.is_aligned()

    def align(self):
        self.engage()

    @state(first=True)
    def inital_state(self):
        self.target = None
        self.aimed_at_angle = None

        self.pos_history.enable()

        self.cv_enabled = True

        self.next_state('moving_to_position')

    @state
    def moving_to_position(self):
        if self._move_to_position():
            self.next_state('done')

    def done(self):
        super().done()

        self.pos_history.disable()

        self.cv_enabled = False
