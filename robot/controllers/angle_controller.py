
import math

import hal

from magicbot import tunable
from robotpy_ext.common_drivers.navx.ahrs import AHRS

from components.swervedrive import SwerveDrive
from .my_pid_base import BasePIDComponent

class AngleController(BasePIDComponent):
    '''
        When enabled, controls the angle of the robot
    '''
    
    drive = SwerveDrive

    kP = tunable(0.0004)
    kI = tunable(0.000)
    kD = tunable(0.008)
    kF = tunable(0.0)
    
    kToleranceDegrees = tunable(3.5)
    kIzone = tunable(3.5)
    
    def __init__(self):
        
        self.ahrs = AHRS.create_spi()
        
        super().__init__(self.ahrs.getYaw, 'angle_ctrl')
        
        if hasattr(self, 'pid'):
            self.pid.setInputRange(-180.0,  180.0)
            self.pid.setOutputRange(-1.0, 1.0)
            self.pid.setContinuous(True)
        
        self.report = 0
        
    def get_angle(self):
        """Returns the robot's current heading"""
        return self.ahrs.getYaw()
       
    def align_to(self, angle):
        """Moves the robot and turns it to a specified absolute direction"""
        #print(angle)
        self.setpoint = angle

    def is_aligned(self):
        """
            Returns True if robot is pointing at specified angle.
            
            .. note:: Always returns False when move_at_angle is not being
                      called.
        """
        if not self.enabled:
            return False
        
        angle = self.get_angle()
        setpoint = self.setpoint
        
        # compensate for wraparound (code from PIDController)
        error = setpoint - angle
        if abs(error) > 180.0:
            if error > 0:
                error = error - 360.0
            else:
                error = error + 360.0
        
        print(abs(error))
        return abs(error) < self.kToleranceDegrees

    def reset_angle(self):
        self.ahrs.reset()
    
    def compute_error(self, setpoint, pid_input):
        error = setpoint - pid_input
        if abs(error) > 180.0:
            if error > 0:
                error = error - 360.0
            else:
                error = error + 360.0
                
        return error
    
    def execute(self):
        super().execute()
        
        # rate will never be None when the component is not enabled
        if self.rate is not None:
            if self.is_aligned():
                self.drive.set_raw_rcw(0)
            else:
                print('Setting rotation: %s' % self.rate)
                self.drive.set_raw_rcw(self.rate)
                
