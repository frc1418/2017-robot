import math

from networktables.util import ntproperty
import wpilib
import ctre


class PhysicsEngine:
    
    rr_degrees = ntproperty("/SmartDashboard/drive/rr_module/degrees", 0)
    lr_degrees = ntproperty("/SmartDashboard/drive/rl_module/degrees", 0)
    rf_degrees = ntproperty("/SmartDashboard/drive/fr_module/degrees", 0)
    lf_degrees = ntproperty("/SmartDashboard/drive/fl_module/degrees", 0)

    def __init__(self, controller):
        self.controller = controller

        self.controller.add_device_gyro_channel('navxmxp_spi_4_angle')



    def initialize(self, hal_data):
        self.rr_rotate_encoder = 0
        self.rl_rotate_encoder = 0
        self.fr_rotate_encoder = 0
        self.fl_rotate_encoder = 0

    """
        Update pyfrc simulator

        Keyword arguments:
        self -- Global dictionary of everything.
        hal_data -- Data about motors and other components.
        now -- Current time in ms
        tm_diff -- Diff between current time and time when last checked
    """
    def update_sim(self, hal_data, now, tm_diff):
        

        try:
            #print(hal_data['pwm'][3]['value'])
            self.rr_rotate_encoder += hal_data['pwm'][3]['value'] * tm_diff * 20
            self.rl_rotate_encoder += hal_data['pwm'][1]['value'] * tm_diff * 20
            self.fr_rotate_encoder += hal_data['pwm'][2]['value'] * tm_diff * 20
            self.fl_rotate_encoder += hal_data['pwm'][0]['value'] * tm_diff * 20
            
            self.rr_rotate_encoder %= 5
            self.rl_rotate_encoder %= 5
            self.fr_rotate_encoder %= 5
            self.fl_rotate_encoder %= 5
            
            hal_data['analog_in'][0]['avg_voltage'] = self.rr_rotate_encoder
            hal_data['analog_in'][2]['avg_voltage'] = self.rl_rotate_encoder
            hal_data['analog_in'][1]['avg_voltage'] = self.fr_rotate_encoder
            hal_data['analog_in'][3]['avg_voltage'] = self.fl_rotate_encoder
            #print('---\nRR_ENC:%s\n' % hal_data['analog_in'][0]['avg_voltage'])
        except:
            pass

        try:
            #RR Motor
            hal_data['CAN'][30]['enc_position'] -= hal_data['CAN'][30]['value'] / 1023 * tm_diff * 1000000
            #RL Motor
            hal_data['CAN'][20]['enc_position'] -= hal_data['CAN'][20]['value'] / 1023 * tm_diff * 1000000
            #FR Motor
            hal_data['CAN'][10]['enc_position'] -= hal_data['CAN'][10]['value'] / 1023 * tm_diff * 1000000
            #FL Motor
            hal_data['CAN'][5]['enc_position'] -= hal_data['CAN'][5]['value'] / 1023 * tm_diff * 1000000
            #print(rr_motor)

        except:
            pass
        
        rr_motor = hal_data['CAN'][30]['value'] / 1023
        lr_motor = -hal_data['CAN'][20]['value'] / 1023
        rf_motor = hal_data['CAN'][10]['value'] / 1023
        lf_motor = -hal_data['CAN'][5]['value'] / 1023
        
        #def four_motor_swerve_drivetrain(lr_motor, rr_motor, lf_motor, rf_motor, lr_angle, rr_angle, lf_angle, rf_angle, x_wheelbase=2, y_wheelbase=2, speed=5):
        vx, vy, vw = four_motor_swerve_drivetrain(lr_motor, rr_motor, lf_motor, rf_motor, self.lr_degrees, self.rr_degrees, self.lf_degrees, self.rf_degrees, x_wheelbase = 3, y_wheelbase = 3.6, speed=9)
        self.controller.vector_drive(vx, vy, vw, tm_diff)
        
def four_motor_swerve_drivetrain(lr_motor, rr_motor, lf_motor, rf_motor, lr_angle, rr_angle, lf_angle, rf_angle, x_wheelbase=2, y_wheelbase=2, speed=5):
    '''
        Four motors that can be rotated in any direction
        
        If any motors are inverted, then you will need to multiply that motor's
        value by -1.
        
        :param lr_motor:   Left rear motor value (-1 to 1); 1 is forward
        :param rr_motor:   Right rear motor value (-1 to 1); 1 is forward
        :param lf_motor:   Left front motor value (-1 to 1); 1 is forward
        :param rf_motor:   Right front motor value (-1 to 1); 1 is forward
        :param lr_angle:   Left rear motor angle in degrees (0 to 360)
        :param rr_angle:   Right rear motor angle in degrees (0 to 360)
        :param lf_angle:   Left front motor angle in degrees (0 to 360)
        :param rf_angle:   Right front motor angle in degrees (0 to 360)
        :param x_wheelbase: The distance in feet between right and left wheels.
        :param y_wheelbase: The distance in feet between forward and rear wheels.
        :param speed:      Speed of robot in feet per second (see above)
        
        :returns: Speed of robot in x (ft/s), Speed of robot in y (ft/s), 
                  clockwise rotation of robot (radians/s)
    '''
    # Calculate speed of each wheel
    lr = lr_motor * speed
    rr = rr_motor * speed
    lf = lf_motor * speed
    rf = rf_motor * speed

    # Calculate angle in radians
    lr_rad = lr_angle * (math.pi / 180)
    rr_rad = rr_angle * (math.pi / 180)
    lf_rad = lf_angle * (math.pi / 180)
    rf_rad = rf_angle * (math.pi / 180)

    # Calculate wheelbase radius
    wheelbase_radius = math.sqrt(((x_wheelbase/2) ** 2) + (((y_wheelbase/2) ** 2)))

    # Calculates the Vx and Vy components
    # Sin an Cos inverted because forward is 0 on swerve wheels
    Vx = (math.sin(lr_rad) * lr) + (math.sin(rr_rad) * rr) + (math.sin(lf_rad) * lf) + (math.sin(rf_rad) * rf) 
    Vy = (math.cos(lr_rad) * lr) + (math.cos(rr_rad) * rr) + (math.cos(lf_rad) * lf) + (math.cos(rf_rad) * rf)
    
    # To make left negative
    Vx *= -1
    
    # Adjusts the angle corresponding to a diameter that is perpendicular to the radius (add or subtract 45deg)
    lr_rad = (lr_rad + (math.pi / 4)) % (2 * math.pi)
    rr_rad = (rr_rad - (math.pi / 4)) % (2 * math.pi)
    lf_rad = (lf_rad - (math.pi / 4)) % (2 * math.pi)
    rf_rad = (rf_rad + (math.pi / 4)) % (2 * math.pi)

    # Finds the rotational velocity by finding the torque and adding them up
    Vw = wheelbase_radius * ((math.cos(lr_rad) * lr) + (math.cos(rr_rad) * -rr) + (math.cos(lf_rad) * lf) + (math.cos(rf_rad) * -rf))
    
    Vx *= 0.25
    Vy *= 0.25
    Vw *= 0.25

    return Vx, Vy, Vw

