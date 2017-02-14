from robotpy_ext.autonomous import state, timed_state, StatefulAutonomous
from components import swervedrive
from controllers.pos_controller import XPosController, YPosController
from controllers.angle_controller import AngleController, MovingAngleController
from controllers.position_history import PositionHistory

import wpilib
from networktables import NetworkTable
from magicbot.magic_tunable import tunable

class DriveTest(StatefulAutonomous):
    MODE_NAME = 'Drive_Test'
    DEFAULT = False

    drive = swervedrive.SwerveDrive

    @timed_state(duration = 2, first = True)
    def drive_test(self, initial_call):
        # Go forward
        if initial_call:
            self.drive.enable_position_prediction()
        self.drive.set_raw_fwd(0.5)
        
class DriveStraightTest(StatefulAutonomous):
    MODE_NAME = 'Drive_Straight_Test'
    DEFAULT = False
    
    drive = swervedrive.SwerveDrive
    y_ctrl = YPosController
    
    drive_distance_feet = tunable(5) #Feet
        
    
    @timed_state(duration=10.0, first=True, next_state='failed_distance')
    def drive_distance(self, initial_call):
        if initial_call:
            self.sd = NetworkTable.getTable("SmartDashboard")
            self.drive.enable_position_prediction()
        
        self.y_ctrl.move_to(self.drive_distance_feet)
        self.sd.putNumber("TESTING: ", self.y_ctrl.get_position())
        
        if self.y_ctrl.is_at_location():
            self.next_state('done')
            
    @state
    def failed_distance(self):
        self.next_state('done')
        
    @state
    def done(self):
        self.drive.disable_position_prediction()

class DriveLeftTest(StatefulAutonomous):
    MODE_NAME = 'Drive_Left_Test'
    DEFAULT = False
    
    drive = swervedrive.SwerveDrive
    x_ctrl = XPosController
    
    drive_distance_feet = tunable(-5) #Feet
        
    
    @timed_state(duration=10.0, first=True, next_state='failed_distance')
    def drive_distance(self, initial_call):
        if initial_call:
            self.sd = NetworkTable.getTable("SmartDashboard")
            self.drive.enable_position_prediction()
        
        self.x_ctrl.move_to(self.drive_distance_feet)
        
        if self.x_ctrl.is_at_location():
            self.next_state('done')
            
    @state
    def failed_distance(self):
        self.next_state('done')
        
    @state
    def done(self):
        self.drive.disable_position_prediction()
        
        
class GyroTest(StatefulAutonomous):
    MODE_NAME = 'Gyro_Test'
    DEFAULT = False
    
    drive = swervedrive.SwerveDrive
    angle_ctrl = AngleController
    
    align_to = tunable(-30) #Deg
    
    @timed_state(duration=10.0, first=True, next_state='failed_align')
    def align(self, initial_call):
        if initial_call:
            self.angle_ctrl.reset_angle()
            
            self.drive.allow_reverse = True
            self.drive.wait_for_align = True
            
            self.drive.threshold_input_vectors = False
            
        #print('About to set angle')
        self.angle_ctrl.align_to(self.align_to)
        
        if self.angle_ctrl.is_aligned():
            self.next_state('done')
            
    @state
    def failed_align(self):
        self.next_state('done')
        
    @state
    def done(self):
        self.drive.set_raw_rcw(0.0)
        self.drive.wait_for_align = False
        self.drive.threshold_input_vectors = True
        self.drive.disable_position_prediction()

class DriveStraightWithGyroTest(StatefulAutonomous):
    MODE_NAME = 'Drive_Straight_With_Gyro_Test'
    DEFAULT = False
    
    drive = swervedrive.SwerveDrive
    y_ctrl = YPosController
    moving_angle_ctrl = MovingAngleController
    
    drive_distance_feet = tunable(5) #Feet
    align_to = tunable(0) #Deg
        
    
    @timed_state(duration=10.0, first=True, next_state='failed_distance')
    def drive_distance(self, initial_call):
        if initial_call:
            self.drive.field_centric = True
            self.moving_angle_ctrl.reset_angle()
            self.drive.enable_position_prediction()
        
        self.moving_angle_ctrl.align_to(self.align_to)
        self.y_ctrl.move_to(self.drive_distance_feet)
        
        if self.y_ctrl.is_at_location():
            self.next_state('done')
            
    @state
    def failed_distance(self):
        self.next_state('done')
        
    @state
    def done(self):
        self.drive.disable_position_prediction()

class AlignAndPlace(StatefulAutonomous):
    MODE_NAME = 'Align_and_place'
    DEFAULT = False
    
    drive = swervedrive.SwerveDrive
    y_ctrl = YPosController
    x_ctrl = XPosController
    angle_ctrl = AngleController
    pos_history = PositionHistory
    
    @timed_state(duration=10.0, first=True, next_state='failed_distance')
    def drive_distance(self, initial_call):
        if initial_call:
            self.drive.enable_position_prediction()
            self.pos_history.enable()
            
    @state
    def failed_distance(self):
        self.next_state('done')
        
    @state
    def done(self):
        self.drive.set_raw_rcw(0.0)
        self.drive.disable_position_prediction()
        
    
    