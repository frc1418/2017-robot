#!/usr/bin/env python3

import magicbot
import wpilib
import ctre

from robotpy_ext.control.button_debouncer import ButtonDebouncer
from networktables.util import ntproperty

from robotpy_ext.common_drivers import navx

from networktables.networktable import NetworkTable

from components import shooter, swervemodule, swervedrive


class MyRobot(magicbot.MagicRobot):

    shooter = shooter.Shooter
    drive = swervedrive.SwerveDrive

    def createObjects(self):
        """Create basic components (motor controllers, joysticks, etc.)"""
        # NavX (purple board on top of the RoboRIO)
        self.navx = navx.AHRS.create_spi()

        # Initialize SmartDashboard
        self.sd = NetworkTable.getTable('SmartDashboard')

        # Joysticks
        self.joystick1 = wpilib.Joystick(0)
        self.joystick2 = wpilib.Joystick(1)

        # Motors
        self.rr_module = swervemodule.SwerveModule(ctre.CANTalon(10), wpilib.VictorSP(1),wpilib.AnalogInput(1), SDPrefix="rr_module", zero=0.0, inverted=True)
        self.rl_module = swervemodule.SwerveModule(ctre.CANTalon(15), wpilib.VictorSP(2),wpilib.AnalogInput(2), SDPrefix="rl_module", zero=0.0)
        self.fr_module = swervemodule.SwerveModule(ctre.CANTalon(20), wpilib.VictorSP(3),wpilib.AnalogInput(3), SDPrefix="fr_module", zero=0.0, inverted=True)
        self.fl_module = swervemodule.SwerveModule(ctre.CANTalon(25), wpilib.VictorSP(4),wpilib.AnalogInput(4), SDPrefix="fl_module", zero=0.0)

        self.shooter_motor = ctre.CANTalon(5)

        # TODO: Drivetrain object

    def autonomous(self):
        """Prepare for autonomous mode"""
        pass

    def disabledPeriodic(self):
        """Repeat periodically while robot is disabled. Usually emptied.
        Sometimes used to easily test sensors and other things."""
        pass

    def disabledInit(self):
        """Do once right away when robot is disabled."""

    def teleopInit(self):
        """Do when teleoperated mode is started."""

    def teleopPeriodic(self):
        """Do periodically while robot is in teleoperated mode."""
        
        self.drive.move(self.joystick1.getY()*-1, self.joystick1.getX()*-1, self.joystick2.getX()*-1)
        
        # Temporary: Spin shooter motor based on joystick1's Y.
        self.shooter.rpm = self.joystick1.getY() * 1024

if __name__ == '__main__':
    wpilib.run(MyRobot)
