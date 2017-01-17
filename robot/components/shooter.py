import wpilib
import ctre
from networktables.networktable import NetworkTable

class Shooter:
    shooter_motor = ctre.CANTalon

    def __init__(self):
        self.sd = NetworkTable.getTable('SmartDashboard')

        self.rpm = 0

    def on_enable(self):
        # Set control mode of motor to Velocity control so we can pass an RPM it should turn at
        self.shooter_motor.changeControlMode(ctre.CANTalon.ControlMode.Speed)

    def execute(self):
        """Repeating code"""
        # Set the motor to the currently-desired RPM
        self.shooter_motor.set(self.rpm)
        # Set requested RPM back to zero. This way the robot will stop if it crashes.
        self.rpm = 0

    def update_sd(self, name):
        """Puts refreshed values to SmartDashboard"""
        pass
