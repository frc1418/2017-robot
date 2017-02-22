
import hal
import wpilib
import time
import threading

from controllers.angle_controller import AngleController
from components import swervedrive
from collections import deque

class PositionHistory:
    
    angle_ctrl = AngleController
    
    
    def __init__(self):
        
        self.running = True
        self.last_ts = None
        self.enabled = False
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        
        # limited buffer of data
        self.buffer = deque(maxlen=20)
        
        # WPILib sleep/etc functions have more overhead, so only use in simulation
        if hal.HALIsSimulation():
            self.delay = wpilib.Timer.delay
            self.get_now = wpilib.Timer.getFPGATimestamp
        else:
            self.delay = time.sleep
            self.get_now = time.time
            
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        
        wpilib.Resource._add_global_resource(self)
        
    def free(self):
        with self.cond:
            self.running = False
            self.cond.notify()
        
        self.thread.join()
        
    def enable(self):
        # edge triggered, don't call continuously
        with self.cond:
            self.buffer.clear()
            self.last_ts = None
            self.enabled = True
            self.cond.notify()
    
    def disable(self):
        with self.lock:
            self.enabled = False
            self.last_ts = None
    
    def get_position(self, ts):
        
        #return self.angle_ctrl.get_angle(), self.distance_ctrl.get_position(), None
        
        if self.last_ts is not None:
            with self.lock:
                offset = round((self.last_ts - ts)/0.050)
                if offset < len(self.buffer):
                    return self.buffer[offset]
    
    def _run(self):
        
        while True:
            
            with self.cond:
                while not self.enabled:
                    self.cond.wait()
                    if self.running == False:
                        return
        
            while self.enabled:
                
                now = self.get_now()
                angle = self.angle_ctrl.get_angle()
                x = self.x_ctrl.get_position()
                y = self.y_ctrl.get_position()
                
                with self.lock:
                    if self.enabled:
                        self.buffer.appendleft((angle, x, y, now))
                        self.last_ts = now
                
                # Not very precise, but we're keeping timestamps
                # so it's probably ok.. 
                delay = max(0.050 - (self.get_now() - now), 0.001)
                self.delay(delay)
    
    def execute(self):
        pass