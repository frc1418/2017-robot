from robotpy_ext.autonomous import state, timed_state, StatefulAutonomous
from components import swervedrive, gearpicker
from controllers.pos_controller import XPosController, YPosController
from controllers.angle_controller import AngleController, MovingAngleController
from controllers.position_history import PositionHistory

import wpilib
from networktables import NetworkTable
from magicbot.magic_tunable import tunable

class RightSideGearPlace(StatefulAutonomous):
    'Place robot 15in from string 90deg to string'
    MODE_NAME = "Right side gear place"
    DEFAULT = True
    
    DIRECTION = 1

    drive = swervedrive.SwerveDrive
    gear_picker = gearpicker.GearPicker
    
    
    
    x_ctrl = XPosController
    y_ctrl = YPosController
    angle_ctrl = AngleController
    moving_angle_ctrl = MovingAngleController
    
    out_distance = tunable(7.1)
    rotate_to_angle = tunable(-60)
    wiggle_value = tunable(-5)
    to_gear_distance = tunable(3)
    drive_back_distance = tunable(-3)
    drive_past_line_distance = tunable(5)

    @timed_state(duration = 7, next_state="failed", first = True)
    def drive_out(self, initial_call):
        # Go forward
        if initial_call:
            self.gear_picker._picker_state = 2
            self.angle_ctrl.reset_angle()
            self.drive.enable_position_prediction()
        
        self.y_ctrl.move_to(self.out_distance)
        self.moving_angle_ctrl.align_to(0)
        
        if self.y_ctrl.is_at_location():
            self.next_state("rotate")
    
    @timed_state(duration = 5, next_state="failed")
    def rotate(self):
        self.angle_ctrl.align_to(self.rotate_to_angle * self.DIRECTION)
        
        if self.angle_ctrl.is_aligned():
            self.next_state("drive_to_gear")
            
    @timed_state(duration = 3, next_state='rcw_with_gear')
    def drive_to_gear(self, initial_call):
        if initial_call:
            self.drive.reset_position_prediction()
            
        self.y_ctrl.move_to(self.to_gear_distance)
        self.moving_angle_ctrl.align_to(self.rotate_to_angle * self.DIRECTION)
        
        if self.y_ctrl.is_at_location():
            self.gear_picker._picker_state = 1
            self.next_state('drive_back')
    
    @timed_state(duration = 1, next_state = 'try_release')
    def rcw_with_gear(self):
        self.y_ctrl.move_to(self.to_gear_distance)
        self.drive.set_raw_rcw(0.4 * self.DIRECTION)
        
        if self.y_ctrl.is_at_location():
            self.drive.set_raw_rcw(0.0)
            self.gear_picker._picker_state = 1
            self.next_state('drive_back')
    
    @state
    def try_release(self):
        self.gear_picker._picker_state = 1
        self.next_state('drive_back')
            
    @timed_state(duration = 5, next_state='failed')
    def drive_back(self, initial_call):
        if initial_call:
            self.drive.reset_position_prediction()
            
        self.y_ctrl.move_to(self.drive_back_distance)
        self.moving_angle_ctrl.align_to(self.rotate_to_angle * self.DIRECTION)
        
        if self.y_ctrl.is_at_location():
            self.next_state('rotate_back')
    
    @state
    def rotate_back(self):
        self.angle_ctrl.align_to(0)
        
        if self.angle_ctrl.is_aligned():
            self.next_state('drive_past_line')
    
    @state
    def drive_past_line(self, initial_call):
        if initial_call:
            self.drive.reset_position_prediction()
        
        self.moving_angle_ctrl.align_to(0)
        self.y_ctrl.move_to(self.drive_past_line_distance)
            
        if self.y_ctrl.is_at_location():
            self.next_state('done')

    @state
    def failed(self):
        self.next_state("done")
    
    @state
    def done(self):
        pass
    
class LeftSideGearPlace(RightSideGearPlace):
    'Place robot 15in from string 90deg to string'
    MODE_NAME = "Left side gear place"
    DEFAULT = False
    
    DIRECTION = -1
    
class MiddleGearPlace(StatefulAutonomous):
    MODE_NAME = "Middle Gear Place"
    DEFAULT = False
    
    drive = swervedrive.SwerveDrive
    gear_picker = gearpicker.GearPicker
    
    x_ctrl = XPosController
    y_ctrl = YPosController
    angle_ctrl = AngleController
    moving_angle_ctrl = MovingAngleController
    
    out_distance = tunable(6)
    drive_back_distance = tunable(-3)
    strafe_distance = tunable(8)
    drive_past_line_distance = tunable(8)
    
    @timed_state(duration = 7, next_state="failed", first = True)
    def drive_out(self, initial_call):
        # Go forward
        if initial_call:
            self.gear_picker._picker_state = 2
            self.angle_ctrl.reset_angle()
            self.drive.enable_position_prediction()
        
        self.y_ctrl.move_to(self.out_distance)
        self.moving_angle_ctrl.align_to(0)
        
        if self.y_ctrl.is_at_location():
            self.gear_picker._picker_state = 1
            self.next_state("drive_back")
            
    @timed_state(duration = 5, next_state='failed')
    def drive_back(self, initial_call):
        if initial_call:
            self.drive.reset_position_prediction()
            
        self.y_ctrl.move_to(self.drive_back_distance)
        self.moving_angle_ctrl.align_to(0)
        
        if self.y_ctrl.is_at_location():
            self.next_state('rotate_back')
    
    @timed_state(duration = 6, next_state='failed')
    def strafe_distance(self, initial_call):
        if initial_call:
            self.drive.reset_position_prediction()
            
        self.x_ctrl.move_to(self.drive_back_distance)
        self.moving_angle_ctrl.align_to(0)
        
        if self.x_ctrl.is_at_location():
            self.next_state('drive_past_line')
            
    @timed_state(duration = 6, next_state='failed')
    def drive_past_line(self, initial_call):
        if initial_call:
            self.drive.reset_position_prediction()
            
        self.y_ctrl.move_to(self.drive_past_line_distance)
        self.moving_angle_ctrl.align_to(0)
        
        if self.y_ctrl.is_at_location():
            self.next_state('done')
    
    @state
    def failed(self):
        self.next_state("done")
    
    @state
    def done(self):
        pass