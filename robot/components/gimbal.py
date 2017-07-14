from common import scale
from networktables.networktable import NetworkTable
from networktables.util import ntproperty
import wpilib


class Gimbal:
    # Yaw = left/right, pitch = up/down
    gimbal_yaw = wpilib.Servo
    gimbal_pitch = wpilib.Servo

    _yaw = ntproperty('/camera/gimbal/yaw', 0)
    _pitch = ntproperty('/camera/gimbal/pitch', 0.52)

    def setup(self):
        self.sd = NetworkTable.getTable('SmartDashboard')

    def on_enable(self):
        pass

    @property
    def yaw(self):
        pass

    @property
    def pitch(self):
        pass

    @yaw.setter
    def yaw(self, value):
        self._yaw = scale.scale(value, -1, 1, 0, 0.14)

    @pitch.setter
    def pitch(self, value):
        self._pitch = scale.scale(value, -1, 1, 0.18, 0.72)

    def execute(self):
        """
        Repeating code.
        """
        self.gimbal_yaw.set(self._yaw)
        self.gimbal_pitch.set(self._pitch)

    def update_sd(self, name):
        """
        Put refreshed values to SmartDashboard.
        """
        pass
