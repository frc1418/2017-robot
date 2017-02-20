#!/usr/bin/env python3

import magicbot
import wpilib
import ctre

from robotpy_ext.control.button_debouncer import ButtonDebouncer
from networktables.util import ntproperty

from robotpy_ext.common_drivers import navx

from networktables.networktable import NetworkTable

from components import shooter, gearpicker, swervemodule, swervedrive, climber, gimbal

from common import pressure_sensor, scale

from controllers.pos_controller import XPosController, YPosController
from controllers.angle_controller import AngleController, MovingAngleController
from controllers.position_history import PositionHistory

class MyRobot(magicbot.MagicRobot):

    drive = swervedrive.SwerveDrive
    shooter = shooter.Shooter
    gear_picker = gearpicker.GearPicker
    climber = climber.Climber
    gimbal = gimbal.Gimbal
    
    y_ctrl = YPosController
    x_ctrl = XPosController
    angle_ctrl = AngleController
    moving_angle_ctrl = MovingAngleController
    
    pos_history = PositionHistory

    def createObjects(self):
        """Create basic components (motor controllers, joysticks, etc.)"""
        # NavX
        self.navx = navx.AHRS.create_spi()

        # Initialize SmartDashboard
        self.sd = NetworkTable.getTable('SmartDashboard')

        # Joysticks
        self.left_joystick = wpilib.Joystick(0)
        self.right_joystick = wpilib.Joystick(1)
        
        self.secondary_joystick = wpilib.Joystick(2)

        # Triggers and buttons
        self.right_trigger = ButtonDebouncer(self.right_joystick, 1)
        self.secondary_trigger = ButtonDebouncer(self.secondary_joystick, 1)

        # Drive motors
        self.rr_module = swervemodule.SwerveModule(ctre.CANTalon(30), wpilib.VictorSP(3), wpilib.AnalogInput(0), SDPrefix='rr_module', zero=1.94, has_drive_encoder=True)
        self.rl_module = swervemodule.SwerveModule(ctre.CANTalon(20), wpilib.VictorSP(1), wpilib.AnalogInput(2), SDPrefix='rl_module', zero=0.58, inverted = True)
        self.fr_module = swervemodule.SwerveModule(ctre.CANTalon(10), wpilib.VictorSP(2), wpilib.AnalogInput(1), SDPrefix='fr_module', zero=3.76)
        self.fl_module = swervemodule.SwerveModule(ctre.CANTalon(5), wpilib.VictorSP(0), wpilib.AnalogInput(3), SDPrefix='fl_module', zero=4.03, has_drive_encoder=True, inverted = True)

        # Drive control
        self.field_centric_button = ButtonDebouncer(self.left_joystick, 6)
        self.predict_position = ButtonDebouncer(self.left_joystick, 7)

        # Shooting motors
        self.shooter_motor = ctre.CANTalon(15)
        self.belt_motor = wpilib.spark.Spark(9)

        self.intake_motor = wpilib.VictorSP(8)

        # Pistons for gear picker
        self.picker = wpilib.DoubleSolenoid(6, 7)
        self.pivot = wpilib.DoubleSolenoid(4, 5)
        
        self.pessure_sensor = pressure_sensor.REVAnalogPressureSensor(navx.pins.getNavxAnalogInChannel(0))

        # Toggling button on secondary joystick
        self.pivot_toggle_button = ButtonDebouncer(self.secondary_joystick, 2)

        # Or, up and down buttons on right joystick
        self.pivot_down_button = ButtonDebouncer(self.right_joystick, 2)
        self.pivot_up_button = ButtonDebouncer(self.right_joystick, 3)

        self.accumulator_button = ButtonDebouncer(self.secondary_joystick, 5)
        self.accumulator_button2 = ButtonDebouncer(self.left_joystick, 8)

        # Climb motors
        self.climb_motor1 = wpilib.spark.Spark(4)
        self.climb_motor2 = wpilib.spark.Spark(5)

        # Camera gimble
        self.gimbal_yaw = wpilib.Servo(6)
        self.gimbal_pitch = wpilib.Servo(7)

        # PDP
        self.pdp = wpilib.PowerDistributionPanel(0)

    def robotInit(self):
        super().robotInit()
        
        wpilib.CameraServer.launch('camera/vision.py:main')

    def autonomous(self):
        """Prepare for autonomous mode."""
        self.drive.allow_reverse = False
        self.drive.wait_for_align = True
        self.drive.threshold_input_vectors = True
        
        magicbot.MagicRobot.autonomous(self)

    def disabledPeriodic(self):
        """
        Repeat periodically while robot is disabled.

        Usually emptied.
        Sometimes used to easily test sensors and other things.
        """
        self.update_sd()


    def disabledInit(self):
        """Do once right away when robot is disabled."""

    def teleopInit(self):
        """Do when teleoperated mode is started."""
        self.drive.prepare_for_teleop() #This is a poor solution to the drive system maintain speed/direction
        
        self.drive._field_centric = True # Doesn't set the property becuase the property resets the navx
        self.drive.allow_reverse = False
        self.drive.wait_for_align = False
        self.drive.threshold_input_vectors = True
        
        self.drive.disable_position_prediction()

    def teleopPeriodic(self):
        """Do periodically while robot is in teleoperated mode."""
        
        #Drive system
        self.drive.move(self.left_joystick.getY()*-1, self.left_joystick.getX()*-1, self.right_joystick.getX()*-1)

        if self.field_centric_button.get():
            self.drive.field_centric = not self.drive.field_centric
            
        if self.left_joystick.getRawButton(2):
            self.drive.xy_multiplier = 0.5
            self.drive.rotation_multiplier = 0.25
        else:
            self.drive.xy_multiplier = 1.0
            self.drive.rotation_multiplier = 0.75
            
        if self.right_joystick.getRawButton(4):
            self.drive.set_raw_strafe(0.25)
        
        if self.right_joystick.getRawButton(5):
            self.drive.set_raw_strafe(-0.25)

        # Gear picker
        if self.pivot_toggle_button.get():
            if self.gear_picker._pivot_state == 1:
                self.gear_picker.pivot_down()
            else:
                self.gear_picker.pivot_up()
                        
        if self.pivot_up_button.get():
            self.gear_picker.pivot_up()
        elif self.pivot_down_button.get():
            self.gear_picker.pivot_down()

        if self.right_trigger.get() or self.secondary_trigger.get():
            self.gear_picker.actuate_picker()

        # Climber
        if self.left_joystick.getRawButton(3) or self.secondary_joystick.getRawButton(4):
            self.climber.climb()
            
        # Shooter
        if self.left_joystick.getRawButton(1) or self.secondary_joystick.getRawButton(3):
            self.shooter.shoot()
        else:
            self.shooter.stop()
            
        # Accumulator
        if self.accumulator_button2.get() or self.accumulator_button.get():
            self.gear_picker.intake_on = not self.gear_picker.intake_on
            
        # Secondary driver gimble control
        
        if self.secondary_joystick.getRawButton(12):
            #(input, input_min, input_max, output_min, output_max)
            self.gimbal.yaw = scale.scale(self.secondary_joystick.getX()*-1, -1, 1, 0, 0.14)
            self.gimbal.pitch = scale.scale(self.secondary_joystick.getY()*-1, -1, 1, 0.18, 0.72)
            
        self.update_sd()

    def update_sd(self):
        self.sd.putNumber('current/climb1_current_draw', self.pdp.getCurrent(1))
        self.sd.putNumber('current/climb2_current_draw', self.pdp.getCurrent(2))
        self.sd.putNumber('current/rr_rotate_current_draw', self.pdp.getCurrent(8))
        self.sd.putNumber('current/fl_rotate_current_draw', self.pdp.getCurrent(7))
        
        self.sd.putNumber('pneumatics/tank_pressure', self.pessure_sensor.getPressure())
        
        self.drive.update_smartdash()

if __name__ == '__main__':
    wpilib.run(MyRobot)
