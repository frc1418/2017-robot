#!/usr/bin/env python3

import magicbot
import wpilib
import ctre

from robotpy_ext.control.button_debouncer import ButtonDebouncer
from networktables.util import ntproperty

from robotpy_ext.common_drivers import navx

from networktables.networktable import NetworkTable

from components import shooter, gearpicker, swervemodule, swervedrive, climber, gimbal

from common import pressure_sensor, scale, toggle_button

from controllers.pos_controller import XPosController, YPosController, FCXPosController, FCYPosController
from controllers.angle_controller import AngleController, MovingAngleController
from controllers.position_history import PositionHistory
from controllers.field_centric import FieldCentric
from controllers.position_tracker import PositionTracker, FCPositionTracker
from controllers.auto_align import AutoAlign
from magicbot.magic_tunable import tunable


class MyRobot(magicbot.MagicRobot):

    drive = swervedrive.SwerveDrive
    shooter = shooter.Shooter
    gear_picker = gearpicker.GearPicker
    climber = climber.Climber
    gimbal = gimbal.Gimbal

    field_centric = FieldCentric

    tracker = PositionTracker
    fc_tracker = FCPositionTracker

    y_ctrl = YPosController
    x_ctrl = XPosController

    fc_y_ctrl = FCYPosController
    fc_x_ctrl = FCXPosController

    angle_ctrl = AngleController
    moving_angle_ctrl = MovingAngleController

    pos_history = PositionHistory
    auto_align = AutoAlign

    gamepad_mode = tunable(False)

    def createObjects(self):
        """
        Create basic components (motor controllers, joysticks, etc.).
        """
        # NavX
        self.navx = navx.AHRS.create_spi()

        # Initialize SmartDashboard
        self.sd = NetworkTable.getTable('SmartDashboard')

        # Joysticks
        self.left_joystick = wpilib.Joystick(0)
        self.right_joystick = wpilib.Joystick(1)

        self.secondary_joystick = wpilib.Joystick(2)

        # Triggers and buttons
        self.secondary_trigger = ButtonDebouncer(self.secondary_joystick, 1)

        # Drive motors
        self.fr_module = swervemodule.SwerveModule(ctre.CANTalon(30), wpilib.VictorSP(3), wpilib.AnalogInput(0), sd_prefix='rr_module', zero=1.85, has_drive_encoder=True)
        self.fl_module = swervemodule.SwerveModule(ctre.CANTalon(20), wpilib.VictorSP(1), wpilib.AnalogInput(2), sd_prefix='rl_module', zero=3.92, inverted=True)
        self.rr_module = swervemodule.SwerveModule(ctre.CANTalon(10), wpilib.VictorSP(2), wpilib.AnalogInput(1), sd_prefix='fr_module', zero=4.59)
        self.rl_module = swervemodule.SwerveModule(ctre.CANTalon(5), wpilib.VictorSP(0), wpilib.AnalogInput(3), sd_prefix='fl_module', zero=2.44, has_drive_encoder=True, inverted=True)

        # Drive control
        self.field_centric_button = ButtonDebouncer(self.left_joystick, 6)
        self.predict_position = ButtonDebouncer(self.left_joystick, 7)

        self.field_centric_drive = True

        self.field_centric_hot_switch = toggle_button.TrueToggleButton(self.left_joystick, 1)

        self.left_shimmy = toggle_button.TrueToggleButton(self.right_joystick, 4)
        self.right_shimmy = toggle_button.TrueToggleButton(self.right_joystick, 5)

        self.align_button = toggle_button.TrueToggleButton(self.right_joystick, 10)

        # Shooting motors
        self.shooter_motor = ctre.CANTalon(15)
        self.belt_motor = wpilib.spark.Spark(9)

        self.light_controller = wpilib.VictorSP(8)

        # Pistons for gear picker
        self.picker = wpilib.DoubleSolenoid(6, 7)
        self.pivot = wpilib.DoubleSolenoid(4, 5)

        self.pessure_sensor = pressure_sensor.REVAnalogPressureSensor(navx.pins.getNavxAnalogInChannel(0))

        # Toggling button on secondary joystick
        self.pivot_toggle_button = ButtonDebouncer(self.secondary_joystick, 2)

        # Or, up and down buttons on right joystick
        self.pivot_down_button = ButtonDebouncer(self.right_joystick, 2)
        self.pivot_up_button = ButtonDebouncer(self.right_joystick, 3)

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
        """
        Prepare for autonomous mode.
        """
        self.field_centric.set_raw_values = True
        self.drive.allow_reverse = False
        self.drive.wait_for_align = True
        self.drive.threshold_input_vectors = True

        super().autonomous()

    def disabledPeriodic(self):
        """
        Repeat periodically while robot is disabled.

        Usually emptied.
        Sometimes used to easily test sensors and other things.
        """
        self.update_sd()

    def disabledInit(self):
        """
        Do once right away when robot is disabled.
        """

    def teleopInit(self):
        """
        Do when teleoperated mode is started.
        """
        self.drive.flush()  # This is a poor solution to the drive system maintain speed/direction

        self.field_centric.set_raw_values = False
        self.drive.allow_reverse = True
        self.drive.wait_for_align = False
        self.drive.squared_inputs = True
        self.drive.threshold_input_vectors = True

    def move(self, x, y, rcw):
        if self.right_joystick.getRawButton(1):
                rcw *= 0.75

        if not self.field_centric_drive or self.left_joystick.getRawButton(1):
            self.drive.move(x, y, rcw)
        else:
            self.field_centric.move(x, y)
            self.drive.set_rcw(rcw)

    def teleopPeriodic(self):
        """
        Do periodically while robot is in teleoperated mode.
        """

        # Drive system
        if not self.gamepad_mode or self.ds.isFMSAttached():
            self.move(self.left_joystick.getY()*-1, self.left_joystick.getX()*-1, self.right_joystick.getX())
        else:
            self.move(self.left_joystick.getRawAxis(1)*-1, self.left_joystick.getRawAxis(0), self.left_joystick.getRawAxis(2)*-1)

        if self.field_centric_button.get():
            if not self.field_centric_drive:
                self.navx.reset()
            self.field_centric_drive = not self.field_centric_drive

        if self.left_joystick.getRawButton(2):
            self.drive.request_wheel_lock = True

        if self.right_joystick.getRawButton(4):
            self.drive.set_raw_strafe(0.25)
        elif self.right_joystick.getRawButton(5):
            self.drive.set_raw_strafe(-0.25)
        if self.right_joystick.getRawButton(3):
            self.drive.set_raw_fwd(0.35)
        elif self.right_joystick.getRawButton(2):
            self.drive.set_raw_fwd(-0.35)

        # Gear picker
        if self.pivot_toggle_button.get():
            if self.gear_picker._pivot_state == 1:
                self.gear_picker.pivot_down()
            else:
                self.gear_picker.pivot_up()

        if self.secondary_trigger.get():
            self.gear_picker.actuate_picker()

        # Climber
        if self.left_joystick.getRawButton(3) or self.secondary_joystick.getRawButton(4):
            self.climber.climb(-1)
        if self.secondary_joystick.getRawButton(6):
            self.climber.climb(-0.5)

        if self.secondary_joystick.getRawButton(5):
            self.light_controller.set(1)
        else:
            self.light_controller.set(0)

        # Shooter
        if self.secondary_joystick.getRawButton(3):
            self.shooter.shoot()
        else:
            self.shooter.stop()

        # Secondary driver gimble control
        if self.secondary_joystick.getRawButton(12):
            # scale.scale params: (input, input_min, input_max, output_min, output_max)
            self.gimbal.yaw = scale.scale(self.secondary_joystick.getX()*-1, -1, 1, 0, 0.14)
            self.gimbal.pitch = scale.scale(self.secondary_joystick.getY()*-1, -1, 1, 0.18, 0.72)

        # Auto align test
        if self.right_joystick.getRawButton(10):
            self.auto_align.align()
        if self.align_button.get_released():
            self.auto_align.done()

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
