import math
from . import swervemodule

from networktables import NetworkTable
from networktables.util import ntproperty

from robotpy_ext.common_drivers import navx

class SwerveDrive:
    fl_module = swervemodule.SwerveModule
    fr_module = swervemodule.SwerveModule
    rl_module = swervemodule.SwerveModule
    rr_module = swervemodule.SwerveModule
    
    navx = navx.AHRS
    
    snap_rotation_axes = ntproperty('/SmartDashboard/drive/drive/snap_rotation_axes', 8)
    lower_input_thresh = ntproperty('/SmartDashboard/drive/drive/lower_input_thresh', 0.1)
    
    rotation_multiplier = ntproperty('/SmartDashboard/drive/drive/rotation_multiplier', 1)
    xy_multiplier = ntproperty('/SmartDashboard/drive/drive/xy_multiplier', 1)
    
    debugging = ntproperty('/SmartDashboard/drive/drive/debugging', False)    
        
    def setup(self):
        '''
        Called after injection
        '''
        
        #Put all the modules into a dictionary  
        self.modules = {
                'front_right': self.fr_module,
                'front_left': self.fl_module,
                'rear_left': self.rl_module,
                'rear_right': self.rr_module
        }
        
        self.sd = NetworkTable.getTable('SmartDashboard')
        
        self._requested_vectors = {
                'fwd': 0,
                'strafe': 0,
                'rcw': 0
        }
        
        self._requested_angles = {
                'front_right': 0,
                'front_left': 0,
                'rear_left': 0,
                'rear_right': 0
        }
        
        self._requested_speeds = {
                'front_right': 0,
                'front_left': 0,
                'rear_left': 0,
                'rear_right': 0
        }
        
        self.field_centric = False
        self.allow_reverse = True
        self.squared_inputs = True
        self.snap_rotation = False
        
        self.width = 1
        self.length = 1
        
    @property
    def chassis_dimension(self):
        return (self.width, self.length)
    
    @chassis_dimension.setter
    def chassis_dimension(self, dimension):
        self.width = dimension[0]
        self.length = dimension[1]

    @property
    def allow_reverse(self):
        return self._allow_reverse
    
    @allow_reverse.setter
    def allow_reverse(self, value):
        self._allow_reverse = value
        
        for module in self.modules.values():
            module.allow_reverse = value
    
    @property
    def field_centric(self):
        return self._field_centric
    
    @field_centric.setter
    def field_centric(self, value):
        if self.navx is None:
            self._field_centric = False
        else:
            self._field_centric = value
        
    @staticmethod
    def square_input(input):
        return math.copysign(input * input, input)
    
    @staticmethod
    def normalize(data):
        maxMagnitude = max(abs(x) for x in data)
        if maxMagnitude > 1.0:
            for i in range(len(data)):
                data[i] = data[i] / maxMagnitude
                
        return data
    
    @staticmethod
    def normalizeDictionary(data):
        maxMagnitude = max(abs(x) for x in data.values())
        if maxMagnitude > 1.0:
            for key in data:
                data[key] = data[key] / maxMagnitude
        return data
    
    def move(self, fwd, strafe, rcw):
        '''
        Calulates the speed and angle for each wheel given the requested movement
        :param fwd: the requested movement in the Y direction 2D plane
        :param strafe: the requested movement in the X direction of the 2D plane
        :param rcw: the requestest magnatude of the rotational vector of a 2D plane
        '''
        
        if self.squared_inputs:
            fwd = self.square_input(fwd)
            strafe = self.square_input(strafe)
            rcw = self.square_input(rcw)
            
            fwd, strafe, rcw = self.normalize([fwd, strafe, rcw])
        
        fwd *= self.xy_multiplier
        strafe *= self.xy_multiplier
        rcw *= self.rotation_multiplier
        
        #Locks the wheels to certain intervals if locking is true TODO: Convert math to radians
        if self.snap_rotation:
            interval = 360/self.snap_rotation_axes
            half_interval = interval/2
            
            #Calclulates the radius (speed) from the give x and y
            r = math.sqrt((fwd ** 2) + (strafe ** 2))
            
            #Gets the degree from the given x and y
            deg = math.degrees(math.atan2(fwd, strafe)) 
            
            #Corrects the degree to one of 8 axes
            remainder = deg % interval            
            if remainder >= half_interval:
                deg += interval - remainder
            else:
                deg -= remainder
                
            #Gets the fwd/strafe values out of the new deg
            theta = math.radians(deg)
            fwd = math.sin(theta)*r
            strafe = math.cos(theta)*r
            
        
        if self.field_centric:
            theta = math.radians(360-self.navx.yaw)
            
            fwdX = fwd * math.cos(theta)
            fwdY = (-fwd) * math.sin(theta) #TODO: verify and understand why fwd is neg
            strafeX = strafe * math.cos(theta)
            strafeY = strafe * math.sin(theta)
            
            fwd = fwdX + strafeY
            strafe = fwdY + strafeX
            
        self._requested_vectors['fwd'] = fwd
        self._requested_vectors['strafe'] = strafe
        self._requested_vectors['rcw'] = rcw
        
    def _calculate_vectors(self):
        '''
        Calculates the requested speed and angle of each modules from self._requested_vectors and stores them in
        self._requested_speeds and self._requested_speeds dictionaries. 
        '''
        
        #Does nothing if the values are lower than the input thresh
        if abs(self._requested_vectors['fwd']) < self.lower_input_thresh:
           self._requested_vectors['fwd'] = 0;
        
        if abs(self._requested_vectors['strafe']) < self.lower_input_thresh:
            self._requested_vectors['strafe'] = 0;
        
        if abs(self._requested_vectors['rcw']) < self.lower_input_thresh:
            self._requested_vectors['rcw'] = 0;
            
        if self._requested_vectors['rcw'] != 0 and self._requested_vectors['strafe'] != 0 and self._requested_vectors['fwd'] != 0: #Prevents a useless loop.
            self._requested_speeds = dict.fromkeys(self._requested_speeds, 0)
            return

        ratio = math.sqrt((self.length ** 2)+(self.width ** 2))
        #Velocities per quadrant
        leftY = self._requested_vectors['fwd'] - (self._requested_vectors['rcw'] * (self.width / ratio))
        rightY = self._requested_vectors['fwd'] + (self._requested_vectors['rcw'] * (self.width / ratio))
        frontX = self._requested_vectors['strafe'] + (self._requested_vectors['rcw'] * (self.length / ratio))
        rearX = self._requested_vectors['strafe'] - (self._requested_vectors['rcw'] * (self.length / ratio))

        #Calculate the speed and angle for each wheel given the combination of the corresponding quadrant vectors
        fr_speed = math.sqrt((rightY ** 2) + (frontX ** 2))
        fr_angle = math.degrees(math.atan2(frontX, rightY))

        fl_speed = math.sqrt((leftY ** 2) + (frontX ** 2))
        fl_angle = math.degrees(math.atan2(frontX, leftY))

        rl_speed = math.sqrt((leftY ** 2) + (rearX ** 2))
        rl_angle = math.degrees(math.atan2(rearX, leftY))

        rr_speed = math.sqrt((rightY ** 2) + (rearX ** 2))
        rr_angle = math.degrees(math.atan2(rearX, rightY))

        #Assigns the speeds and angles in dictionaries
        self._requested_speeds['front_right'] = fr_speed
        self._requested_speeds['front_left'] = fl_speed
        self._requested_speeds['rear_right'] = rr_speed
        self._requested_speeds['rear_left'] = rl_speed
        
        self._requested_angles['front_right'] = fr_angle
        self._requested_angles['front_left'] = fl_angle
        self._requested_angles['rear_right'] = rr_angle
        self._requested_angles['rear_left'] = rl_angle
        
        self._requested_speeds = self.normalizeDictionary(self._requested_speeds)

    def execute(self):
        '''
        Sends the speeds and angles to each corresponding wheel module.
        Executes the doit in each wheel module.
        '''
        self.update_smartdash()
        
        
        self._calculate_vectors()
        for key in self.modules:
            self.modules[key].move(self._requested_speeds[key], self._requested_angles[key])
            
        self._requested_speeds = dict.fromkeys(self._requested_speeds, 0)

        for key in self.modules:
            self.modules[key].execute()

    def update_smartdash(self):
        '''
        Pushes some internal variables for debugging.
        '''
        self.sd.putNumber("drive/drive/navx_yaw", self.navx.yaw)
        self.sd.putBoolean("drive/drive/field_centric", self.field_centric)
        self.sd.putBoolean("drive/drive/allow_reverse", self.allow_reverse)
        self.sd.putBoolean("drive/drive/snap_rotation", self.snap_rotation)
            
        if self.debugging:
            for key in self._requested_angles:
                self.sd.putNumber("drive/drive/%s angle" % key, self._requested_angles[key])
                self.sd.putNumber("drive/drive/%s speed" % key, self._requested_speeds[key])
                