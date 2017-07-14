from .base_auto import VictisAuto
from components import swervedrive, gearpicker, shooter
from controllers import angle_controller, pos_controller, position_tracker
from magicbot.magic_tunable import tunable
from magicbot.state_machine import timed_state


class SideGearPlace(VictisAuto):
    MODE_NAME = 'Drive Forward'
    DEFAULT = False

    # Injection
    drive = swervedrive.SwerveDrive
    gear_picker = gearpicker.GearPicker
    shooter = shooter.Shooter

    x_ctrl = pos_controller.XPosController
    y_ctrl = pos_controller.YPosController

    fc_y_ctrl = pos_controller.FCYPosController
    fc_x_ctrl = pos_controller.FCXPosController

    angle_ctrl = angle_controller.AngleController
    moving_angle_ctrl = angle_controller.MovingAngleController

    tracker = position_tracker.PositionTracker
    fc_tracker = position_tracker.FCPositionTracker

    s_direction = tunable(1, subtable='side')  # When set to 1 this will run gear place on the right side

    # Distances
    # Warning: this is a field centric mode; all positions are relative to starting position
    s_out_y = tunable(12, subtable='side')
    s_out_x = tunable(0, subtable='side')

    #################################################
    # This portion of the code is for placing gears #
    #################################################

    @timed_state(duration=10, next_state='failed', first=True)
    def drive_out(self):
        self.fc_y_ctrl.move_to(self.s_out_y)
        self.fc_x_ctrl.move_to(self.s_out_x)

        self.moving_angle_ctrl.align_to(0)

        if self.fc_y_ctrl.is_at_location() and self.fc_x_ctrl.is_at_location():
            self.next_state('finish')
