
from magicbot import state, StateMachine, tunable
from networktables.util import ntproperty

from components.swervedrive import SwerveDrive
from controllers.pos_controller import XPosController, YPosController
from controllers.angle_controller import AngleController
from controllers.position_history import PositionHistory

class AutoAlign(StateMachine):
    drive = SwerveDrive
    
    x_ctrl = XPosController
    y_ctrl = YPosController
    angle_ctrl = AngleController
    
    pos_history = PositionHistory
    
    cv_enabled = ntproperty('/camera/control/cv_enabled', False)
    
    ideal_skew = tunable(-0.967)
    ideal_angle = tunable(-1.804)
    
    # found, time, angle, skew
    target = ntproperty('/camera/target', (0.0, 0.0, float('inf'), float('inf')))
    
    def __init__(self):
        
        self.lasttime = 0
        self.aimed_at_angle = None
        self.aimed_at_x = None
        
    def align(self):
        self.engage()
    
    @state(first=True)
    def moving_to_position(self, initial_call):
        
        if initial_call:
            self.aimed_at_angle = None
            self.pos_history.enable()
            self.cv_enabled = True
        
        found, time, offset, skew = self.target
        
        # do I have new information?
        if self.lasttime < time:
            history = self.pos_history.get_position(time)
            
            if found > 0 and history is not None:
                r_angle, r_x, r_y, r_time = history 
                
                self.aimed_at_angle = r_angle + offset - self.ideal_angle
        
            self.lasttime = time
        
        if self.aimed_at_angle is not None:
            self.angle_ctrl.align_to(self.aimed_at_angle)
        
        #return self.angle_ctrl.is_aligned()
    
    def done(self):
        super().done()
        
        self.pos_history.disable()
        
        self.cv_enabled = False