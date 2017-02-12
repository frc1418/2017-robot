import hal
import wpilib

from magicbot import state, timed_state, StateMachine, tunable
from networktables import NetworkTable
from networktables.util import ntproperty

from components.swervedrive import SwerveDrive
from controllers.pos_controller import XPosController, YPosController
from controllers.angle_controller import AngleController

class AutoPlace(StateMachine):
    drive = SwerveDrive
    
    x_ctrl = XPosController
    y_ctrl = YPosController
    angle_ctrl = AngleController
    
    pos_history = PositionHistory
    
    camera_enabled = ntproperty('/camera/enabled', False)
    
    ideal_skew = tunable(-0.967)
    ideal_angle = tunable(-1.804)
    
    def __init__(self):
        
        target = None
        
        nt = NetworkTable.getTable('/camera')
        nt.addTableListener(self._on_target, True, 'target')
        
        self.aimed_at_angle = None
        self.aimed_at_x = None
    
    def _on_target(self, source, key, value, isNew):
        self.target = value
        
    def _move_to_position(self):
        target = self.target

        if target is not None and len(target) > 0:
            target = target[:]
            
            angle, skew, capture_ts = target
            history = self.pos_history.get_position(captur_ts)
            
            if history is not None:
                r_angle, r_x, r_y, r_time = history 
                
                self.aimed_at_angle = r_angle + angle - self.ideal_angle
                self.aimed_at_x = r_angle
    