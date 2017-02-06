
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
    
    robot_setpoint = tunable(0)
    robot_angle = tunable(0)

    kP = tunable(0.01)
    kI = tunable(0.000)
    kD = tunable(0.00)
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
        
    '''
    def pidWrite(self, output):
        """This function is invoked periodically by the PID Controller,
        based upon navX MXP yaw angle input and PID Coefficients.
        """
        
        # old wheels
        # x: 0.25 to 0.5
        # y: 0.15 to 0.6
        
        # new wheels @ bbn
        # x: 0.55 to 1.0
        # y: 0.15 to 0.6
        
        # The pure output of the PID controller isn't enough.. 
        # .. need to scale between some min out
        
        # scale input range
        
        # scale between 0 and max
        rotation_rate = math.copysign(abs(output)*0.45+0.55, output)
        super().pidWrite(rotation_rate)'''
    
    def execute(self):
        
        super().execute()
        
        # rate will never be None when the component is not enabled
        if self.rate is not None:
            if self.is_aligned():
                self.drive.set_raw_rcw(0)
            else:
                self.drive.set_raw_rcw(self.rate)
                
        self.report += 1
        if self.report % 5 == 0:
            self.robot_angle = self.get_angle()
            self.robot_setpoint = self.setpoint
