
import math

from magicbot import tunable

from components import swervedrive
from .my_pid_base import BasePIDComponent

class XPosController(BasePIDComponent):
    
    drive = swervedrive.SwerveDrive
    
    kP = tunable(0.05)
    kI = tunable(0.0004)
    kD = tunable(0.0)
    kF = tunable(0.0)
        
    kToleranceFeet = tunable(0.25)
    kIzone = tunable(0.25)
        
    def __init__(self):
        super().__init__(self.get_position, 'x_ctrl')
        
        self.MIN_RAW_OUTPUT = -0.5
        self.MAX_RAW_OUTPUT = 0.5 
        #self.pid.setOutputRange(-1.0, 1.0)
    
    def get_position(self):
        return self.drive.get_predicted_x() / 1.0
    
    def move_to(self, position):
        self.setpoint = position
        
    def is_at_location(self):
        #print('DIST FROM LOCATION: %s' % abs(self.get_position() - self.setpoint))
        return self.enabled and \
                abs(self.get_position() - self.setpoint) < self.kToleranceFeet
    
    def execute(self):
        
        super().execute()
        
        if self.rate is not None:
            if self.is_at_location():
                self.drive.set_raw_strafe(0)
            else:
                self.drive.set_raw_strafe(self.rate)

class YPosController(BasePIDComponent):
        
    drive = swervedrive.SwerveDrive
    
    kP = tunable(0.05)
    kI = tunable(0.0004)
    kD = tunable(0.0)
    kF = tunable(0.0)
        
    kToleranceFeet = tunable(0.25)
    kIzone = tunable(0.25)
        
    def __init__(self):
        super().__init__(self.get_position, 'y_ctrl')  
        
        self.MIN_RAW_OUTPUT = -0.5
        self.MAX_RAW_OUTPUT = 0.5   
    
    def get_position(self):
        return self.drive.get_predicted_y() / 1.0
    
    def move_to(self, position):
        self.setpoint = position
        
    def is_at_location(self):
        #print('DIST FROM LOCATION: %s' % abs(self.get_position() - self.setpoint))
        return self.enabled and abs(self.get_position() - self.setpoint) < self.kToleranceFeet
    
    def execute(self):
        
        super().execute()
        
        if self.rate is not None:
            if self.is_at_location():
                self.drive.set_raw_fwd(0)
            else:
                self.drive.set_raw_fwd(self.rate)

