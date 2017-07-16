"""
NOTE: This swerve drive system is far from ideal. There tidy few
strange things that arise due to our hardware implementation of
swervedrive. 3/4 through build season I started to notice this
but by that point things just needed to work. I told my team many
times that if I were to rewrite swerve drive it would look very 
different than the system here. 

TLDR: This system is designed for the irregularities of ONE robot
and should not be seen as an over-arching example.

- Carter Fendley
"""

import math
from . import swervemodule

from networktables import NetworkTable
from networktables.util import ntproperty

from robotpy_ext.common_drivers import navx


class SwerveDrive:
    """
    This class is the heart of the swerve drive system. Changes in this
    class should be made with EXTERME care. Many different parts of
    robot code use the functions with in this class.
    """
    
    fl_module = swervemodule.SwerveModule
    fr_module = swervemodule.SwerveModule
    rl_module = swervemodule.SwerveModule
    rr_module = swervemodule.SwerveModule

    navx = navx.AHRS

    snap_rotation_axes = ntproperty('/SmartDashboard/drive/drive/snap_rotation_axes', 8)
    lower_input_thresh = ntproperty('/SmartDashboard/drive/drive/lower_input_thresh', 0.06)

    rotation_multiplier = ntproperty('/SmartDashboard/drive/drive/rotation_multiplier', 0.5)
    xy_multiplier = ntproperty('/SmartDashboard/drive/drive/xy_multiplier', 0.85)

    debugging = ntproperty('/SmartDashboard/drive/drive/debugging', False)

    def setup(self):
        """
        Called after injection.
        """

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

        # Variables that allow enabling and disabling of features in code
        self.allow_reverse = False
        self.squared_inputs = True
        self.snap_rotation = False
        self.wait_for_align = False
        self.threshold_input_vectors = True

        self.width = (22/12)/2
        self.length = (18.5/12)/2

        self.request_wheel_lock = False

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

    def flush(self):
        """
        This method should be called to reset all requested values of the drive system.
        """
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
            module.flush()

    def set_raw_fwd(self, fwd):
        """
        Sets the raw fwd value to prevent it from being passed through any filters
        
        :param fwd: A value from -1 to 1
        """
        self._requested_vectors['fwd'] = fwd

    def set_raw_strafe(self, strafe):
        """
        Sets the raw strafe value to prevent it from being passed through any filters
        
        :param strafe: A value from -1 to 1
        """
        self._requested_vectors['strafe'] = strafe

    def set_raw_rcw(self, rcw):
        """
        Sets the raw rcw value to prevent it from being passed through any filters
        
        :param rcw: A value from -1 to 1
        """
        self._requested_vectors['rcw'] = rcw

    def set_fwd(self, fwd):
        """
        Individually sets the fwd value. (passed through filters)
        
        :param fwd: A value from -1 to 1
        """
        if self.squared_inputs:
            fwd = self.square_input(fwd)

        fwd *= self.xy_multiplier

        self._requested_vectors['fwd'] = fwd

    def set_strafe(self, strafe):
        """
        Individually sets the strafe value. (passed through filters)
        
        :param strafe: A value from -1 to 1
        """
        if self.squared_inputs:
            strafe = self.square_input(strafe)

        strafe *= self.xy_multiplier

        self._requested_vectors['strafe'] = strafe

    def set_rcw(self, rcw):
        """
        Individually sets the rcw value. (passed through filters)
        
        :param rcw: A value from -1 to 1
        """
        if self.squared_inputs:
            rcw = self.square_input(rcw)

        rcw *= self.rotation_multiplier

        self._requested_vectors['rcw'] = rcw

    def move(self, fwd, strafe, rcw):
        """
        Calulates the speed and angle for each wheel given the requested movement
        
        Positive fwd value = Forward robot movement
        Negative fwd value = Backward robot movement
        Positive strafe value = Left robot movement
        Negative strafe value = Right robot movement
        
        :param fwd: the requested movement in the Y direction 2D plane
        :param strafe: the requested movement in the X direction of the 2D plane
        :param rcw: the requestest magnatude of the rotational vector of a 2D plane
        """
        self.set_fwd(fwd)
        self.set_strafe(strafe)
        self.set_rcw(rcw)

    def _calculate_vectors(self):
        """
        Calculate the requested speed and angle of each modules from self._requested_vectors and store them in
        self._requested_speeds and self._requested_speeds dictionaries.
        """
        self._requested_vectors['fwd'], self._requested_vectors['strafe'], self._requested_vectors['rcw'] = self.normalize([self._requested_vectors['fwd'], self._requested_vectors['strafe'], self._requested_vectors['rcw']])

        # Does nothing if the values are lower than the input thresh
        if self.threshold_input_vectors:
            if abs(self._requested_vectors['fwd']) < self.lower_input_thresh:
                self._requested_vectors['fwd'] = 0

            if abs(self._requested_vectors['strafe']) < self.lower_input_thresh:
                self._requested_vectors['strafe'] = 0

            if abs(self._requested_vectors['rcw']) < self.lower_input_thresh:
                self._requested_vectors['rcw'] = 0

            if self._requested_vectors['rcw'] == 0 and self._requested_vectors['strafe'] == 0 and self._requested_vectors['fwd'] == 0:  # Prevents a useless loop.
                self._requested_speeds = dict.fromkeys(self._requested_speeds, 0)  # Do NOT reset the wheel angles.

                if self.request_wheel_lock:
                    # This is intended to set the wheels in such a way that it
                    # difficult to push the robot (intended for defence)

                    self._requested_angles['front_right'] = -45
                    self._requested_angles['front_left'] = 45
                    self._requested_angles['rear_left'] = -45
                    self._requested_angles['rear_right'] = 45

                    self.request_wheel_lock = False

                return

        ratio = math.hypot(self.length, self.width)
        # Velocities per quadrant
        leftY = self._requested_vectors['fwd'] - (self._requested_vectors['rcw'] * (self.width / ratio))
        rightY = self._requested_vectors['fwd'] + (self._requested_vectors['rcw'] * (self.width / ratio))
        frontX = self._requested_vectors['strafe'] + (self._requested_vectors['rcw'] * (self.length / ratio))
        rearX = self._requested_vectors['strafe'] - (self._requested_vectors['rcw'] * (self.length / ratio))

        # Calculate the speed and angle for each wheel given the combination of the corresponding quadrant vectors
        fr_speed = math.hypot(rightY, frontX)
        fr_angle = math.degrees(math.atan2(frontX, rightY))

        fl_speed = math.hypot(leftY, frontX)
        fl_angle = math.degrees(math.atan2(frontX, leftY))

        rl_speed = math.hypot(leftY, rearX)
        rl_angle = math.degrees(math.atan2(rearX, leftY))

        rr_speed = math.hypot(rightY, rearX)
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

        # Zero request vectors for saftey reasons
        self._requested_vectors['fwd'] = 0.0
        self._requested_vectors['strafe'] = 0.0
        self._requested_vectors['rcw'] = 0.0

    def debug(self, debug_modules=False):
        """
        Prints debugging information to log
        """
        if self.predict_position:
            print('Position: ', self._predicted_position, '\n')

        if debug_modules:
            for key in self.modules:
                self.modules[key].debug()

        print('Requested values: ', self._requested_vectors, "\n")
        print('Requested angles: ', self._requested_angles, "\n")
        print('Requested speeds: ', self._requested_speeds, "\n")

    def execute(self):
        """
        Sends the speeds and angles to each corresponding wheel module.
        Executes the doit in each wheel module.
        """
        self.update_smartdash()

        if self.predict_position:
            self._predict_position()

        self._calculate_vectors()

        for key in self.modules:
            self.modules[key].move(self._requested_speeds[key], self._requested_angles[key])

        self._requested_speeds = dict.fromkeys(self._requested_speeds, 0)

        for key in self.modules:
            self.modules[key].execute()

    def update_smartdash(self):
        """
        Pushes some internal variables for debugging.
        """
        self.sd.putNumber('drive/drive/navx_yaw', self.navx.yaw)
        self.sd.putBoolean('drive/drive/allow_reverse', self.allow_reverse)
        self.sd.putBoolean('drive/drive/snap_rotation', self.snap_rotation)
        self.sd.putBoolean('drive/drive/predict_position', self.predict_position)

        if self.predict_position:
            self.sd.putNumber('drive/drive/predicted/x', self._predicted_position['strafe'])
            self.sd.putNumber('drive/drive/predicted/y', self._predicted_position['fwd'])
            self.sd.putNumber('drive/drive/predicted/rot', self._predicted_position['rcw'])

        if self.debugging:
            for key in self._requested_angles:
                self.sd.putNumber('drive/drive/%s angle' % key, self._requested_angles[key])
                self.sd.putNumber('drive/drive/%s speed' % key, self._requested_speeds[key])

        for key in self.modules:
            self.modules[key].update_smartdash()
