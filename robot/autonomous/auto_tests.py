'''
This file contains a number of tests that were used to
diagnose issues with the robot's autonomous functions.
Variables in this file can be changed liberally as needed.

These autonomous modes should not be used during
competition unless you know what you are doing. ;) 
'''

from magicbot.state_machine import state, timed_state
from components import swervedrive
from controllers.pos_controller import XPosController, YPosController, FCXPosController, FCYPosController
from controllers.angle_controller import AngleController, MovingAngleController
from controllers.position_history import PositionHistory
from controllers.position_tracker import PositionTracker, FCPositionTracker
from .base_auto import VictisAuto

import wpilib
from networktables import NetworkTable
from magicbot.magic_tunable import tunable


class DriveTest(VictisAuto):
    MODE_NAME = 'Drive_Test'
    DEFAULT = False

    drive = swervedrive.SwerveDrive

    @timed_state(duration=2, first=True)
    def drive_test(self, initial_call):
        # Go forward
        if initial_call:
            self.drive.field_centric = False
            self.drive.enable_position_prediction()
        self.drive.set_raw_fwd(0.5)


class DriveStraightTest(VictisAuto):
    MODE_NAME = 'Drive_Straight_Test'
    DEFAULT = False

    drive = swervedrive.SwerveDrive
    tracker = PositionTracker
    y_ctrl = YPosController

    drive_distance_feet = tunable(5)

    @timed_state(duration=10.0, first=True, next_state='failed_distance')
    def drive_distance(self, initial_call):
        if initial_call:
            self.drive.field_centric = False
            self.sd = NetworkTable.getTable('SmartDashboard')
            self.tracker.enable()

        self.y_ctrl.move_to(self.drive_distance_feet)
        self.sd.putNumber('TESTING: ', self.y_ctrl.get_position())

        if self.y_ctrl.is_at_location():
            self.next_state('finish')

    @state
    def failed_distance(self):
        self.next_state('finish')

    @state
    def finish(self):
        self.tracker.disable()
        self.done()


class DriveLeftTest(VictisAuto):
    MODE_NAME = 'Drive_Left_Test'
    DEFAULT = False

    drive = swervedrive.SwerveDrive
    x_ctrl = XPosController

    drive_distance_feet = tunable(-5)

    @timed_state(duration=10.0, first=True, next_state='failed_distance')
    def drive_distance(self, initial_call):
        if initial_call:
            self.drive.field_centric = False
            self.sd = NetworkTable.getTable('SmartDashboard')
            self.drive.enable_position_prediction()

        self.x_ctrl.move_to(self.drive_distance_feet)

        if self.x_ctrl.is_at_location():
            self.next_state('finish')

    @state
    def failed_distance(self):
        self.next_state('finish')

    @state
    def finish(self):
        self.drive.disable_position_prediction()
        self.done()


class GyroTest(VictisAuto):
    MODE_NAME = 'Gyro_Test'
    DEFAULT = False

    drive = swervedrive.SwerveDrive
    angle_ctrl = AngleController

    align_to = tunable(-30)  # Degrees

    @timed_state(duration=10.0, first=True, next_state='failed_align')
    def align(self, initial_call):
        if initial_call:
            self.angle_ctrl.reset_angle()

            self.drive.allow_reverse = True

            self.drive.threshold_input_vectors = False

        self.angle_ctrl.align_to(self.align_to)

        if self.angle_ctrl.is_aligned():
            self.next_state('finish')

    @state
    def failed_align(self):
        self.next_state('finish')

    @state
    def finish(self):
        self.drive.set_raw_rcw(0.0)
        self.drive.wait_for_align = False
        self.drive.threshold_input_vectors = True
        self.drive.disable_position_prediction()
        self.done()


class DriveStraightWithGyroTest(VictisAuto):
    MODE_NAME = 'Drive_Straight_With_Gyro_Test'
    DEFAULT = False

    drive = swervedrive.SwerveDrive
    fc_y_ctrl = FCYPosController
    fc_x_ctrl = FCXPosController
    moving_angle_ctrl = MovingAngleController

    fc_tracker = FCPositionTracker

    drive_distance_feet = tunable(-5)  # Feet
    align_to = tunable(0)  # Degrees

    @timed_state(duration=10.0, first=True, next_state='failed_distance')
    def drive_distance(self, initial_call):
        if initial_call:
            self.drive.field_centric = True
            self.moving_angle_ctrl.reset_angle()
            self.fc_tracker.enable()

        self.moving_angle_ctrl.align_to(self.align_to)
        self.fc_x_ctrl.move_to(self.drive_distance_feet)

        if self.fc_x_ctrl.is_at_location():
            self.next_state('finish')

    @state
    def failed_distance(self):
        self.next_state('finish')

    @state
    def finish(self):
        self.fc_tracker.disable()
        self.done()


class AlignAndPlace(VictisAuto):
    MODE_NAME = 'Align_and_place'
    DEFAULT = False

    drive = swervedrive.SwerveDrive
    fc_y_ctrl = FCYPosController
    fc_tracker = FCPositionTracker
    x_ctrl = XPosController
    angle_ctrl = AngleController
    pos_history = PositionHistory

    @timed_state(duration=10.0, first=True, next_state='failed_distance')
    def drive_distance(self, initial_call):
        if initial_call:
            self.drive.field_centric = False
            self.drive.enable_position_prediction()
            self.pos_history.enable()

    @state
    def failed_distance(self):
        pass
