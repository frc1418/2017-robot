from robotpy_ext.autonomous import state, timed_state, StatefulAutonomous
from components import swervedrive, gearpicker
from controllers.pos_controller import XPosController, YPosController
from controllers.angle_controller import AngleController
from controllers.position_history import PositionHistory

import wpilib
from networktables import NetworkTable
from magicbot.magic_tunable import tunable

class RightSideGearPlace(StatefulAutonomous):
    'Placed robot 11in to the right (from the driverstation) of the line'
    MODE_NAME = "Right_side_gear_place"
    DEFAULT = True

    drive = swervedrive.SwerveDrive
    gear_picker = gearpicker.GearPicker
    
    x_ctrl = XPosController
    y_ctrl = YPosController
    angle_ctrl = AngleController
    
    out_distance = tunable(7)
    rotate_to_angle = tunable(-62)
    to_gear_distance = tunable(3.5)
    drive_back_distance = tunable(-3.5)

    @timed_state(duration = 5, next_state="failed", first = True)
    def drive_out(self, initial_call):
        # Go forward
        if initial_call:
            self.gear_picker._picker_state = 2
            self.angle_ctrl.reset_angle()
            self.drive.enable_position_prediction()
        
        self.y_ctrl.move_to(self.out_distance)
        self.angle_ctrl.align_to(0)
        
        if self.y_ctrl.is_at_location():
            self.next_state("rotate")
    
    @timed_state(duration = 5, next_state="failed")
    def rotate(self):
        self.angle_ctrl.align_to(self.rotate_to_angle)
        
        if self.angle_ctrl.is_aligned():
            self.next_state("drive_to_gear")
            
    @timed_state(duration=5, next_state='failed')
    def drive_to_gear(self, initial_call):
        if initial_call:
            self.angle_ctrl.reset_angle()
            self.drive.reset_position_prediction()
            
        self.y_ctrl.move_to(self.to_gear_distance)
        self.angle_ctrl.align_to(0)
        
        if self.y_ctrl.is_at_location():
            self.gear_picker._picker_state = 1
            self.next_state('drive_back')
            
    @timed_state(duration = 5, next_state='failed')
    def drive_back(self):
        if initial_call:
            self.angle_ctrl.reset_angle()
            self.drive.reset_position_prediction()
            
        self.y_ctrl.move_to(self.drive_back_distance)
        self.angle_ctrl.align_to(0)
        
        if self.y_ctrl.is_at_location():
            self.next_state('done')

    @state
    def failed(self):
        self.next_state("done")
    
    @state
    def done(self):
        pass