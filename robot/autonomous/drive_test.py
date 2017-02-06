from robotpy_ext.autonomous import state, timed_state, StatefulAutonomous
from components import swervedrive
from controllers.pos_controller import XPosController, YPosController
import wpilib
from networktables import NetworkTable
from magicbot.magic_tunable import tunable
'''
class DriveTest(StatefulAutonomous):
    MODE_NAME = 'Drive_Test'
    DEFAULT = False

    drive = swervedrive.SwerveDrive

    @timed_state(duration = 2, first = True)
    def drive_test(self, initial_call):
        # Go forward
        if initial_call:
            self.drive.enable_position_prediction()
        self.drive.set_raw_fwd(0.5)'''
        
class DriveStraightTest(StatefulAutonomous):
    MODE_NAME = 'Drive_Straight_Test'
    DEFAULT = True
    
    drive = swervedrive.SwerveDrive
    y_controller = YPosController
    
    drive_distance_feet = tunable(5) #Feet
        
    
    @timed_state(duration=1.0, first=True, next_state='failed_distance')
    def drive_distance(self, initial_call):
        if initial_call:
            self.sd = NetworkTable.getTable("SmartDashboard")
            self.drive.enable_position_prediction()
            self.y_controller.enabled = True
        
        self.y_controller.move_to(self.drive_distance_feet)
        self.sd.putNumber("TESTING: ", self.y_controller.get_position())
        
        if self.y_controller.is_at_location():
            self.y_controller.enabled = False
            self.next_state('done')
            
    @state
    def failed_distance(self):
        self.next_state('done')
        
    @state
    def done(self):
        pass
        
    
    