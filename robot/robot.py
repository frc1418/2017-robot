#!/usr/bin/env python3

import magicbot
import wpilib
import ctre

from robotpy_ext.control.button_debouncer import ButtonDebouncer
from networktables.util import ntproperty

from robotpy_ext.common_drivers import navx

from networktables.networktable import NetworkTable

from components import shooter, gearpicker, swervemodule, swervedrive, climber


class MyRobot(magicbot.MagicRobot):

    drive = swervedrive.SwerveDrive
    shooter = shooter.Shooter
    gear_picker = gearpicker.GearPicker
    climber = climber.Climber

    def createObjects(self):
        """Create basic components (motor controllers, joysticks, etc.)"""
        # NavX
        self.navx = navx.AHRS.create_spi()

        # Initialize SmartDashboard
        self.sd = NetworkTable.getTable('SmartDashboard')

        # Joysticks (1 = left, 2 = right)
        self.joystick1 = wpilib.Joystick(0)
        self.joystick2 = wpilib.Joystick(1)

        # Triggers
        self.left_trigger = ButtonDebouncer(self.joystick1, 1)
        self.right_trigger = ButtonDebouncer(self.joystick2, 1)

        # Motors
        self.rr_module = swervemodule.SwerveModule(ctre.CANTalon(30), wpilib.VictorSP(3), wpilib.AnalogInput(0), SDPrefix="rr_module", zero=1.55)
        self.rl_module = swervemodule.SwerveModule(ctre.CANTalon(20), wpilib.VictorSP(1), wpilib.AnalogInput(2), SDPrefix="rl_module", zero=3.03)
        self.fr_module = swervemodule.SwerveModule(ctre.CANTalon(10), wpilib.VictorSP(2), wpilib.AnalogInput(1), SDPrefix="fr_module", zero=3.73)
        self.fl_module = swervemodule.SwerveModule(ctre.CANTalon(5), wpilib.VictorSP(0), wpilib.AnalogInput(3), SDPrefix="fl_module", zero=1.60)

        # Shooting motors
        self.shooter_motor = ctre.CANTalon(15)
        self.belt_motor = wpilib.spark.Spark(9)
        
        self.intake_motor = wpilib.VictorSP(8)

        # Pistons for gear picker
        self.picker = wpilib.DoubleSolenoid(6, 7)
        self.pivot = wpilib.DoubleSolenoid(4, 5)

        self.pivot_down_button = ButtonDebouncer(self.joystick2, 2)
        self.pivot_up_button = ButtonDebouncer(self.joystick2, 3)
        
        #Climb motors
        self.climb_motor1 = wpilib.spark.Spark(4)
        self.climb_motor2 = wpilib.spark.Spark(5)
        
        # Drive control
        self.field_centric_button = ButtonDebouncer(self.joystick1, 6)
        
        
        self.pdp = wpilib.PowerDistributionPanel(0)


    def autonomous(self):
        """Prepare for autonomous mode."""
        pass

    def disabledPeriodic(self):
        """
        Repeat periodically while robot is disabled.

        Usually emptied.
        Sometimes used to easily test sensors and other things.
        """
        self.rr_module.update_smartdash()
        self.rl_module.update_smartdash()
        self.fr_module.update_smartdash()
        self.fl_module.update_smartdash()
        self.update_sd()
        

    def disabledInit(self):
        """Do once right away when robot is disabled."""

    def teleopInit(self):
        """Do when teleoperated mode is started."""
        pass

    def teleopPeriodic(self):
        """Do periodically while robot is in teleoperated mode."""
        self.drive.move(self.joystick1.getY()*-1, self.joystick1.getX()*-1, self.joystick2.getX()*-1)

        if self.pivot_up_button.get():
            self.gear_picker.pivot_up()
        elif self.pivot_down_button.get():
            self.gear_picker.pivot_down()

        if self.right_trigger.get():
            self.gear_picker.actuate_picker()
            
        if self.joystick1.getRawButton(3):
            self.climber.climb()
        
        if self.joystick1.getRawButton(1):
            self.shooter.shoot()
        else:
            self.shooter.stop()
            
        if self.field_centric_button.get():
            self.drive.field_centric = not self.drive.field_centric
            
            
        self.update_sd()
            
    def update_sd(self):
        self.sd.putNumber("/climber/motor1_current_draw", self.pdp.getCurrent(1))
        self.sd.putNumber("/climber/motor2_current_draw", self.pdp.getCurrent(2))

if __name__ == '__main__':
    wpilib.run(MyRobot)
