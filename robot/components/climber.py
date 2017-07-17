import wpilib
from networktables.networktable import NetworkTable

class Climber:
    """
    This class operates the climber on the robot.
    """
    
    climb_motor1 = wpilib.spark.Spark
    climb_motor2 = wpilib.spark.Spark

    def __init__(self):
        self._motor1_speed = 0
        self._motor2_speed = 0

        self.nt = NetworkTable.getTable('SmartDashboard')

    def climb(self, speed):
        """
        Sets the motor speed of each climbing motor
        
        :param speed: The requested speed from -1 to 1
        """
        self._motor1_speed = speed
        self._motor2_speed = speed

    def execute(self):
        self.climb_motor1.set(self._motor1_speed)
        self.climb_motor2.set(self._motor2_speed)

        self._motor1_speed = 0
        self._motor2_speed = 0
