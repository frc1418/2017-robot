import wpilib
from networktables.networktable import NetworkTable

class Climber:
    climb_motor1 = wpilib.spark.Spark
    climb_motor2 = wpilib.spark.Spark

    def __init__(self):
        self._motor1_speed = 0
        self._motor2_speed = 0

        self.nt = NetworkTable.getTable('SmartDashboard')

    def climb(self, speed):
        self._motor1_speed = speed
        self._motor2_speed = speed

    def execute(self):
        self.climb_motor1.set(self._motor1_speed)
        self.climb_motor2.set(self._motor2_speed)

        self._motor1_speed = 0
        self._motor2_speed = 0
