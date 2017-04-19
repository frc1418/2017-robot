from wpilib import Joystick

class Joystick(Joystick):
    
    def __init__(self):
        self.past_axis_values = {}
        
    def getRawAxis(self, axis, max_rate = 0.01):
        value = Joystick.getRawAxis(self, axis)
        
        if axis is not in self.past_axis_value.get_key()
        
        return Joystick.getRawAxis(self, axis)