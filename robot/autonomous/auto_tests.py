from robotpy_ext.autonomous import state, timed_state, StatefulAutonomous
from components import swervedrive
from controllers.pos_controller import XPosController, YPosController
from controllers.angle_controller import AngleController

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
    y_controller = YPosController
    
    drive_distance_feet = tunable(5) #Feet
        
    
    @timed_state(duration=10.0, first=True, next_state='failed_distance')
    def drive_distance(self, initial_call):
        if initial_call:
            self.sd = NetworkTable.getTable("SmartDashboard")
            self.drive.enable_position_prediction()
        
        self.y_controller.move_to(self.drive_distance_feet)
        self.sd.putNumber("TESTING: ", self.y_controller.get_position())
        
        if self.y_controller.is_at_location():
            self.next_state('done')
            
    @state
    def failed_distance(self):
        self.next_state('done')
        
    @state
    def done(self):
        self.drive.disable_position_prediction()
        
        
class GyroTest(StatefulAutonomous):
    MODE_NAME = 'Gyro_Test'
    DEFAULT = True
    
    drive = swervedrive.SwerveDrive
    angle_controller = AngleController
    
    align_to = tunable(45) #Deg
    
    @timed_state(duration=10.0, first=True, next_state='failed_align')
    def align(self, initial_call):
        if initial_call:
            self.angle_controller.reset_angle()
            self.drive.wait_for_align = True
            self.drive.threshold_input_vectors = False
            
        #print('About to set angle')
        self.angle_controller.align_to(self.align_to)
        
        if self.angle_controller.is_aligned():
            self.next_state('done')
            
    @state
    def failed_align(self):
        self.next_state('done')
        
    @state
    def done(self):
        self.drive.wait_for_align = False
        self.drive.threshold_input_vectors = True
        self.drive.disable_position_prediction()

class DriveStraightWithGyroTest(StatefulAutonomous):
    MODE_NAME = 'Drive_Straight_With_Gyro_Test'
    DEFAULT = False
    
    drive = swervedrive.SwerveDrive
    y_controller = YPosController
    angle_controller = AngleController
    
    drive_distance_feet = tunable(5) #Feet
    align_to = tunable(0) #Deg
        
    
    @timed_state(duration=10.0, first=True, next_state='failed_distance')
    def drive_distance(self, initial_call):
        if initial_call:
            self.angle_controller.reset_angle()
            self.drive.enable_position_prediction()
        
        self.angle_controller.align_to(align_to)
        self.y_controller.move_to(self.drive_distance_feet)
        
        if self.y_controller.is_at_location():
            self.next_state('done')
            
    @state
    def failed_distance(self):
        self.next_state('done')
        
    @state
    def done(self):
        self.drive.disable_position_prediction()
        
        
    
    