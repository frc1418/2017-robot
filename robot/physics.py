import math

from networktables.util import ntproperty
import wpilib
import ctre


class PhysicsEngine:

    def __init__(self, controller):
        self.controller = controller

        self.controller.add_device_gyro_channel('navxmxp_spi_4_angle')



    def initialize(self, hal_data):
        self.rr_rotate_encoder = 2
        self.rl_rotate_encoder = 3
        self.fr_rotate_encoder = 4
        self.fl_rotate_encoder = 1

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
            self.rr_rotate_encoder += -hal_data['pwm'][1]['value'] * tm_diff * 1
            self.rl_rotate_encoder += -hal_data['pwm'][2]['value'] * tm_diff * 1
            self.fr_rotate_encoder += -hal_data['pwm'][3]['value'] * tm_diff * 1
            self.fl_rotate_encoder += -hal_data['pwm'][4]['value'] * tm_diff * 1
            
            self.rr_rotate_encoder %= 5
            self.rl_rotate_encoder %= 5
            self.fr_rotate_encoder %= 5
            self.fl_rotate_encoder %= 5
            
            hal_data['analog_in'][1]['voltage'] = self.rr_rotate_encoder
            hal_data['analog_in'][2]['voltage'] = self.rl_rotate_encoder
            hal_data['analog_in'][3]['voltage'] = self.fr_rotate_encoder
            hal_data['analog_in'][4]['voltage'] = self.fl_rotate_encoder
            #print('---\nRR_ENC:%s\n' % hal_data['analog_in'][1]['voltage'])
        except:
            pass

        try:
            rr_motor = -hal_data['CAN'][10]['value'] / 1023
            rl_motor = -hal_data['CAN'][15]['value'] / 1023
            fr_motor = -hal_data['CAN'][20]['value'] / 1023
            fl_motor = -hal_data['CAN'][25]['value'] / 1023


        except:
            pass

