import math

from magicbot import tunable

from components import swervedrive
from .base_pid_controller import BasePIDComponent
from . import field_centric, position_tracker


class XPosController(BasePIDComponent):

    drive = swervedrive.SwerveDrive
    tracker = position_tracker.PositionTracker

    kP = tunable(0.1)
    kI = tunable(0.0)
    kD = tunable(0.0)
    kF = tunable(0.0)

    kToleranceFeet = tunable(0.25)
    kIzone = tunable(0.25)

    def __init__(self):
        super().__init__(self.get_position, 'x_ctrl')

        self.set_abs_output_range(0.16, 0.8)

    def get_position(self):
        return self.tracker.get_x() / 1.0

    def move_to(self, position):
        self.setpoint = position

    def is_at_location(self):
        return self.enabled and \
                abs(self.get_position() - self.setpoint) < self.kToleranceFeet

    def pidWrite(self, output):
        self.rate = -output

    def execute(self):

        super().execute()

        if self.rate is not None:
            if self.is_at_location():
                self.drive.set_raw_strafe(0)
            else:
                self.drive.set_raw_strafe(self.rate)


class YPosController(BasePIDComponent):

    drive = swervedrive.SwerveDrive
    tracker = position_tracker.PositionTracker

    kP = tunable(0.09)
    kI = tunable(0.0)
    kD = tunable(0.0)
    kF = tunable(0.0)

    kToleranceFeet = tunable(0.25)
    kIzone = tunable(0.25)

    def __init__(self):
        super().__init__(self.get_position, 'y_ctrl')

        self.set_abs_output_range(0.16, 0.8)

    def get_position(self):
        return self.tracker.get_y() / 1.0

    def move_to(self, position):
        self.setpoint = position

    def is_at_location(self):
        return self.enabled and abs(self.get_position() - self.setpoint) < self.kToleranceFeet

    def execute(self):
        super().execute()

        if self.rate is not None:
            if self.is_at_location():
                self.drive.set_raw_fwd(0)
            else:
                self.drive.set_raw_fwd(self.rate)


class FCXPosController(XPosController):

    field_centric = field_centric.FieldCentric
    fc_tracker = position_tracker.FCPositionTracker

    def get_position(self):
        return self.fc_tracker.get_x() / 1.0

    def execute(self):

        super().execute()

        if self.rate is not None:
            if self.is_at_location():
                self.field_centric.set_strafe(0)
            else:
                self.field_centric.set_strafe(self.rate)
                #print(self.rate)


class FCYPosController(YPosController):

    field_centric = field_centric.FieldCentric
    fc_tracker = position_tracker.FCPositionTracker

    def get_position(self):
        return self.fc_tracker.get_y() / 1.0

    def execute(self):

        super().execute()

        if self.rate is not None:
            if self.is_at_location():
                self.field_centric.set_fwd(0)
            else:
                self.field_centric.set_fwd(self.rate)
