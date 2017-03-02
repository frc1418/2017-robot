import wpilib

class TrueToggleButton:
    '''Utility designed to allow for true toggle buttons'''
    
    def __init__(self, joystick, button_num):
        '''
        :param joystick:  Joystick object
        :type  joystick:  :class:`wpilib.Joystick` 
        :param buttonnum: Number of button to retrieve
        :type  buttonnum: int
        '''
        
        self.joystick = joystick
        self.button_num = button_num
        
        self.pressed = False
        self.pressed = True
        
        self.pressed_lock = False
        self.released_lock = False
        
    def get(self):
        return self.get_pressed()
    
    def get_pressed(self):
        '''Returns true if the button is pressed for the first time.'''
        
        self.update()
        
        if self.pressed:
            self.pressed = False
            return True
        
        return False
    
    def get_released(self):
        '''Returns true if the button is released for the first time.'''
        
        self.update()
        
        if self.released:
            self.released = False
            return True
        
        return False
    
    def update(self):
        
        if self.joystick.getRawButton(self.button_num) and not self.pressed_lock:
            self.pressed = True
            self.released = False
            
            self.pressed_lock = True
            self.released_lock = False
            
        elif not self.joystick.getRawButton(self.button_num) and not self.released_lock:
            self.pressed = False
            self.released = True
            
            self.pressed_lock = False
            self.released_lock = True