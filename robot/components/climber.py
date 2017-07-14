from networktables.networktable import NetworkTable
import wpilib


class Climber:
    climb_motor1 = wpilib.spark.Spark
    climb_motor2 = wpilib.spark.Spark

    def __init__(self):
        self._motor_speed = 0

        self.nt = NetworkTable.getTable('SmartDashboard')

    def climb(self, speed):
        self._motor_speed = speed

    def execute(self):
        self.climb_motor1.set(self._motor_speed)
        self.climb_motor2.set(self._motor_speed)

        self._motor_speed = 0
