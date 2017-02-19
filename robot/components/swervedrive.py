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
    lower_input_thresh = ntproperty('/SmartDashboard/drive/drive/lower_input_thresh', 0.06)
    
    rotation_multiplier = ntproperty('/SmartDashboard/drive/drive/rotation_multiplier', 1)
    xy_multiplier = ntproperty('/SmartDashboard/drive/drive/xy_multiplier', 1)
    
    debugging = ntproperty('/SmartDashboard/drive/drive/debugging', False)    
    
    # = ntproperty('/SmartDashboard/drive/drive/debugging', False)  
        
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
        
        self._predicted_position = {
                'fwd': 0,
                'strafe': 0,
                'rcw': 0
        }
        
        self.predict_position = False
        
        self.field_centric = False
        self.allow_reverse = False
        self.squared_inputs = True
        self.snap_rotation = False
        self.wait_for_align = False
        self.threshold_input_vectors = True
        
        self.width = (22/12)/2
        self.length = (18.5/12)/2
        
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
            
            if value:
                self.navx.reset()
        
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
    
    def prepare_for_teleop(self):
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
        
        for module in self.modules.values():
            module.prepare_for_teleop()
        
    
    def enable_position_prediction(self, zero_position = True):
        if not self.modules['front_left'].has_drive_encoder or not self.modules['rear_right'].has_drive_encoder:
            if not self.modules['rear_left'].has_drive_encoder or not self.modules['front_right'].has_drive_encoder:
                raise 'Not enough drive encoders to predict position'
        
        self.predict_position = True
        
        if zero_position:
            self._predicted_position['fwd'] = 0.0
            self._predicted_position['strafe'] = 0.0
            self._predicted_position['rcw'] = 0.0
            
            for key in self.modules:
                self.modules[key].zero_drive_encoder()
    
    def disable_position_prediction(self, zero_position = False):
        self.predict_position = False
        
        if zero_position:
            self._predicted_position['fwd'] = 0.0
            self._predicted_position['strafe'] = 0.0
            self._predicted_position['rcw'] = 0.0
            
    def reset_position_prediction(self):
        self._predicted_position['fwd'] = 0.0
        self._predicted_position['strafe'] = 0.0
        self._predicted_position['rcw'] = 0.0
            
    def _predict_position(self):
        #TODO: Clean up this function. its a mess.
        if self.modules['front_left'].has_drive_encoder and self.modules['rear_right'].has_drive_encoder:
            
            #print('Predicting Position!!!!!!')
            fl_dist = self.modules['front_left'].get_drive_encoder_distance()
            fl_theta = swervemodule.SwerveModule.voltage_to_rad(self.modules['front_left'].get_voltage())
            #if abs(fl_dist) < 0.1:
                #return
            self.modules['front_left'].zero_drive_encoder()
            
            
            rr_dist = self.modules['rear_right'].get_drive_encoder_distance()
            rr_theta = swervemodule.SwerveModule.voltage_to_rad(self.modules['rear_right'].get_voltage())
            self.modules['rear_right'].zero_drive_encoder()
            
            radius = math.sqrt(((self.length) ** 2)+((self.width) ** 2))
            
            # degree / 360 = dist / 2 * pi * r
            # C = 2 * pi * r
            # degree / 360 = dist / C
            # degree = 360 * (dist / C)
            # d * (360 / 2 * pi * r) = degree
            
            fl_y = math.cos(fl_theta) * fl_dist
            fl_x = -math.sin(fl_theta) * fl_dist
            fl_rcw = -((math.cos((fl_theta + (math.pi/4)) % (2*math.pi)) * fl_dist) / (2*math.pi*radius)) * 360#TODO: Check rotation math
            
            
            rr_y = math.cos(rr_theta) * rr_dist
            rr_x = -math.sin(rr_theta) * rr_dist
            rr_rcw = -((math.cos((rr_theta + (math.pi/4)) % (2*math.pi)) * rr_dist) / (2*math.pi*radius)) * 360
            
            #raise 'hi'
            self._predicted_position['fwd'] += (fl_y + rr_y) / 2#Halved because measured from two drive encoders
            self._predicted_position['strafe'] += (fl_x + rr_x) / 2
            self._predicted_position['rcw'] += (fl_rcw + rr_rcw) #This should be halved but isn't becuase robots are strange 
            
        if self.modules['front_right'].has_drive_encoder and self.modules['rear_left'].has_drive_encoder:
            fr_dist = self.modules['front_right'].get_drive_encoder_distance()
            fr_theta = swervemodule.SwerveModule.voltage_to_rad(self.modules['front_right'].get_voltage())
            self.modules['front_right'].zero_drive_encoder()
            
            rl_dist = self.modules['rear_left'].get_drive_encoder_distance()
            rl_theta = swervemodule.SwerveModule.voltage_to_rad(self.modules['rear_left'].get_voltage())
            self.modules['rear_left'].zero_drive_encoder()
            
            radius = math.sqrt((self.length ** 2)+(self.width ** 2))
            
            fr_x = math.cos(fr_theta) * fr_dist
            fr_y = math.sin(fr_theta) * fr_dist
            fr_rcw = -((math.cos((fr_theta + (math.pi/4)) % (2*math.pi)) * fr_dist) / (2*math.pi*radius)) * 360 #TODO: Check rotation math
            
            rl_x = math.cos(rl_theta) * rl_dist
            rl_y = math.sin(rl_theta) * rl_dist
            rl_rcw = ((math.cos((rl_theta + (math.pi/4)) % (2*math.pi)) * rl_dist) / (2*math.pi*radius)) * 360
            
            self._predicted_position['fwd'] += (fr_y + rl_y) / 2
            self._predicted_position['strafe'] += (fr_x + rl_x) / 2
            self._predicted_position['rcw'] += (fr_rcw + rl_rcw)
        
    def get_predicted_x(self):
        if not self.predict_position:
            return None
        
        return self._predicted_position['strafe']
    
    def get_predicted_y(self):
        if not self.predict_position:
            return None
        
        return self._predicted_position['fwd']
    
    def get_predicted_theta(self):
        if not self.predict_position:
            return None
        
        return self._predicted_position['rcw']
    
    def set_raw_fwd(self, fwd):
        self._requested_vectors['fwd'] = fwd

    def set_raw_strafe(self, strafe):
        self._requested_vectors['strafe'] = strafe
    
    def set_raw_rcw(self, rcw):
        self._requested_vectors['rcw'] = rcw
    
    def move(self, fwd, strafe, rcw):
        '''
        Calulates the speed and angle for each wheel given the requested movement
        :param fwd: the requested movement in the Y direction 2D plane
        :param strafe: the requested movement in the X direction of the 2D plane
        :param rcw: the requestest magnatude of the rotational vector of a 2D plane
        '''
        '''
        #Zero request vectors for saftey reasons
        self._requested_vectors['fwd'] = 0.0
        self._requested_vectors['strafe'] = 0.0
        self._requested_vectors['rcw'] = 0.0'''
        
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
            fwdY = (-fwd) * math.sin(theta) # TODO: verify and understand why fwd is neg
            strafeX = strafe * math.cos(theta)
            strafeY = strafe * math.sin(theta)
            
            fwd = fwdX + strafeY
            strafe = fwdY + strafeX
        
        
        self._requested_vectors['fwd'] = fwd
        self._requested_vectors['strafe'] = strafe
        self._requested_vectors['rcw'] = rcw
        
        #print(self._requested_vectors)
        
    def _calculate_vectors(self):
        '''
        Calculates the requested speed and angle of each modules from self._requested_vectors and stores them in
        self._requested_speeds and self._requested_speeds dictionaries. 
        '''
        
        #Does nothing if the values are lower than the input thresh
        if self.threshold_input_vectors:
            if abs(self._requested_vectors['fwd']) < self.lower_input_thresh:
               self._requested_vectors['fwd'] = 0
            
            if abs(self._requested_vectors['strafe']) < self.lower_input_thresh:
                self._requested_vectors['strafe'] = 0
            
            if abs(self._requested_vectors['rcw']) < self.lower_input_thresh:
                self._requested_vectors['rcw'] = 0
                    
            if self._requested_vectors['rcw'] == 0 and self._requested_vectors['strafe'] == 0 and self._requested_vectors['fwd'] == 0: # Prevents a useless loop.
                self._requested_speeds = dict.fromkeys(self._requested_speeds, 0) # Do NOT reset the wheel angles.
                return

        ratio = math.sqrt((self.length ** 2)+(self.width ** 2))
        # Velocities per quadrant
        leftY = self._requested_vectors['fwd'] - (self._requested_vectors['rcw'] * (self.width / ratio))
        rightY = self._requested_vectors['fwd'] + (self._requested_vectors['rcw'] * (self.width / ratio))
        frontX = self._requested_vectors['strafe'] + (self._requested_vectors['rcw'] * (self.length / ratio))
        rearX = self._requested_vectors['strafe'] - (self._requested_vectors['rcw'] * (self.length / ratio))

        # Calculate the speed and angle for each wheel given the combination of the corresponding quadrant vectors
        fr_speed = math.sqrt((rightY ** 2) + (frontX ** 2))
        fr_angle = math.degrees(math.atan2(frontX, rightY))

        fl_speed = math.sqrt((leftY ** 2) + (frontX ** 2))
        fl_angle = math.degrees(math.atan2(frontX, leftY))

        rl_speed = math.sqrt((leftY ** 2) + (rearX ** 2))
        rl_angle = math.degrees(math.atan2(rearX, leftY))

        rr_speed = math.sqrt((rightY ** 2) + (rearX ** 2))
        rr_angle = math.degrees(math.atan2(rearX, rightY))

        # Assigns the speeds and angles in dictionaries
        self._requested_speeds['front_right'] = fr_speed
        self._requested_speeds['front_left'] = fl_speed
        self._requested_speeds['rear_right'] = rr_speed
        self._requested_speeds['rear_left'] = rl_speed
        
        self._requested_angles['front_right'] = fr_angle
        self._requested_angles['front_left'] = fl_angle
        self._requested_angles['rear_right'] = rr_angle
        self._requested_angles['rear_left'] = rl_angle
        
        self._requested_speeds = self.normalizeDictionary(self._requested_speeds)
        
        #print(self._requested_vectors)
        
        
        #Zero request vectors for saftey reasons
        self._requested_vectors['fwd'] = 0.0
        self._requested_vectors['strafe'] = 0.0
        self._requested_vectors['rcw'] = 0.0
        
    def debug(self, debug_modules = False):
        if self.predict_position:
            print("Poistion: ", self._predicted_position, "\n")
        
        if debug_modules:
            for key in self.modules:
                self.modules[key].debug()
        
        print("Requested values: ", self._requested_vectors, "\n")
        print("Requested angles: ", self._requested_angles, "\n")
        print("Requested speeds: ", self._requested_speeds, "\n")

    def execute(self):
        '''
        Sends the speeds and angles to each corresponding wheel module.
        Executes the doit in each wheel module.
        '''
        self.update_smartdash()
        
        if self.predict_position:
            self._predict_position()
            
            #print('Predicting:')
            #print(self._predicted_position)
            #print('\n')
        
        self._calculate_vectors()
        
        #print("Requested speeds: %s" % self._requested_speeds)
        
        all_aligned = True
        if self.wait_for_align:
            for key in self.modules:
                if not self.modules[key].is_aligned_to(self._requested_angles[key]):
                    all_aligned = False
                    break
        if all_aligned:
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
        self.sd.putBoolean("drive/drive/predict_position", self.predict_position)
        
        if self.predict_position:
            self.sd.putNumber("drive/drive/predicted/x", self._predicted_position['strafe'])
            self.sd.putNumber("drive/drive/predicted/y", self._predicted_position['fwd'])
            self.sd.putNumber("drive/drive/predicted/rot", self._predicted_position['rcw'])
            
        if self.debugging:
            for key in self._requested_angles:
                self.sd.putNumber("drive/drive/%s angle" % key, self._requested_angles[key])
                self.sd.putNumber("drive/drive/%s speed" % key, self._requested_speeds[key])
                
        for key in self.modules:
            self.modules[key].update_smartdash()
                