import wpilib
import math

from networktables import NetworkTable
from networktables.util import ntproperty


MAX_VOLTAGE = 5
MAX_TICK = 4096
MAX_DEG = 360

WHEEL_DIAMETER = 4/12 # 4 Inches
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * math.pi
WHEEL_TICKS_PER_REV = 55000

class SwerveModule:
    
    def __init__(self, *args, **kwargs):
        '''
        Swerve drive module was written for a swerve drive train that uses absolute encoders for tracking wheel rotation.
        
        When positional arguments are used:
            :param driveMotor: Motor object
            :param rotateMotor: Motor object
            :param encoder: AnalogInput wpilib object
        
        When key word arguments are used:
            :param encoderPort: analog in port number of Absolute Encoder
            
            :param SDPrefix: a string used to differentiate modules when debugging
            :param zero: The default zero for the encoder
            :param inverted: boolean to invert the wheel rotation
            
            :param allow_reverse: If true allows wheels to spin backwards instead of rotating
            
        '''

        # SmartDashboard
        self.sd = NetworkTable.getTable('SmartDashboard')
        self.sd_prefix = kwargs.pop("SDPrefix", False)

        # Motors
        self.driveMotor = args[0]
        self.driveMotor.setInverted(kwargs.pop("inverted", False))
        
        self.rotateMotor = args[1]
        
        self._requested_voltage = 0 #Angle in voltage form
        self._requested_speed = 0
        
        # Encoder
        self.encoder = args[2]
        self.encoder_zero = kwargs.pop("zero", 0.0)

        # PID
        self._pid_controller = wpilib.PIDController(1.5, 0.0, 0.0, self.encoder, self.rotateMotor)
        self._pid_controller.setContinuous()
        self._pid_controller.setInputRange(0.0, 5.0)
        self._pid_controller.enable()

        # State variables
        self.allow_reverse = kwargs.pop("allow_reverse", True)
        self.debugging = self.sd.getAutoUpdateValue('drive/%s/debugging' % self.sd_prefix, False);
        
        self.has_drive_encoder = kwargs.pop("has_drive_encoder", False)
        self.drive_encoder_zero = 0

    def get_voltage(self):
        return self.encoder.getVoltage() - self.encoder_zero
    
    def get_drive_encoder_tick(self):
        if not self.has_drive_encoder:
            return False
        
        return self.driveMotor.getPosition() - self.drive_encoder_zero
    
    def get_drive_encoder_distance(self):
        if not self.has_drive_encoder:
            return False
        
        return ((self.driveMotor.getPosition() - self.drive_encoder_zero) * WHEEL_CIRCUMFERENCE) / WHEEL_TICKS_PER_REV
    
    def zero_drive_encoder(self):
        if not self.has_drive_encoder:
            return False
        
        self.drive_encoder_zero = self.driveMotor.getPosition()
    
    def prepare_for_teleop(self):
        self._requested_voltage = self.encoder.getVoltage()
        self._requested_speed = 0
    
    @staticmethod
    def voltage_to_degrees(voltage):
        '''
        Converts a given voltage value to degrees

        :param voltage: a voltage value between 0 and 5
        '''

        deg = (voltage/5)*360
        deg = deg

        if deg < 0:
            deg = 360+deg;

        return deg

    @staticmethod
    def voltage_to_rad(voltage):
        '''
        Converts a given voltage value to rad

        :param voltage: a voltage value between 0 and 5
        '''

        rad = (voltage/5)*2*math.pi
        return rad

    @staticmethod
    def voltage_to_tick(voltage):
        '''
        Converts a given voltage value to tick

        :param voltage: a voltage value between 0 and 5
        '''

        return (voltage/5)*4050

    @staticmethod
    def degree_to_voltage(degree):
        '''
        Converts a given degree to voltage

        :param degree: a degree value between 0 and 360
        '''

        return (degree/360)*5
    
    @staticmethod
    def tick_to_voltage(tick):
        '''
        Converts a given tick to voltage

        :param tick: a tick value between 0 and 4050
        '''
        
        return (tick/4050)*5

    def zero_encoder(self):
        '''
        Sets the zero to the current voltage output
        '''

        self.encoder_zero = self.encoder.getVoltage()
        
    def is_aligned_to(self, deg):
        deg = deg % 360
        voltage = ((self.degree_to_voltage(deg)+self.encoder_zero) % 5)
        
        if abs(voltage - self.get_voltage()) < 0.1:
            return True
        return False

    def _set_deg(self, value):
        '''
        Rounds the value to within 360. Sets the requested rotate position (requested voltage).
        Intended to be used only by the move function.
        '''
        
        self._requested_voltage = ((self.degree_to_voltage(value)+self.encoder_zero) % 5)

    def move(self, speed, deg):
        '''
        Sets the requested speed and rotation of passed
        '''
        
        
        
        deg = deg % 360 #Prevents values past 360
        
        if self.allow_reverse:
            if abs( deg - self.voltage_to_degrees(self.get_voltage()) ) > 90:
                speed *= -1
                deg += 180
                
                deg = deg % 360 
                
        self._requested_speed = speed
        self._set_deg(deg)
    
    def debug(self):
        print(self.sd_prefix, "; requested_speed: ", self._requested_speed, " requested_voltage: ", self._requested_voltage)

    def execute(self):
        '''
        Uses the PID controller to get closer to the requested position.
        Sets the speed requested of the drive motor.

        Should be called every robot iteration/loop.
        '''
        
        self._pid_controller.setSetpoint(self._requested_voltage)
        self.driveMotor.set(self._requested_speed)
        
        self._requested_speed = 0.0
            
            
        self.update_smartdash()

    def update_smartdash(self):
        '''
        Outputs a bunch on internal variables for debugging purposes.
        '''
        
        self.sd.putNumber("drive/%s/degrees" % self.sd_prefix, self.voltage_to_degrees(self.get_voltage()))
        
        if self.has_drive_encoder:
            self.sd.putNumber("drive/%s/raw drive position" % self.sd_prefix, self.driveMotor.getPosition())
        
        if self.debugging.value:
            self.sd.putNumber("drive/%s/requested_voltage" % self.sd_prefix, self._requested_voltage)
            self.sd.putNumber("drive/%s/requested_speed" % self.sd_prefix, self._requested_speed)
            self.sd.putNumber("drive/%s/raw voltage" % self.sd_prefix, self.encoder.getVoltage()) # DO NOT USE self.get_voltage() here
            self.sd.putNumber("drive/%s/encoder_zero" % self.sd_prefix, self.encoder_zero)
    
            self.sd.putNumber("drive/%s/PID" % self.sd_prefix, self._pid_controller.get())
            ##self.sd.putNumber("drive/%s/PID Error" % self.sd_prefix, self._pid_controller.getError())
            
        
            
            self.sd.putBoolean("drive/%s/allow_reverse" % self.sd_prefix, self.allow_reverse)
