import wpilib

from networktables.networktable import NetworkTable
from networktables.util import ntproperty


class Gimbal:
    # Yaw = left/right, pitch = up/down
    gimbal_yaw = wpilib.Servo
    gimbal_pitch = wpilib.Servo

    yaw = ntproperty('/camera/gimbal/yaw', 0)
    pitch = ntproperty('/camera/gimbal/pitch', 0.52)

    def setup(self):
        self.sd = NetworkTable.getTable('SmartDashboard')

    def on_enable(self):
        pass

    def execute(self):
        """
        Repeating code.
        """
        self.gimbal_yaw.set(self.yaw)
        self.gimbal_pitch.set(self.pitch)

    def update_sd(self, name):
        """
        Put refreshed values to SmartDashboard.
        """
        pass
