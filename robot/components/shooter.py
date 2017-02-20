from magicbot import StateMachine, timed_state, state
import wpilib
import ctre
from magicbot.magic_tunable import tunable

class Shooter(StateMachine):
    belt_motor = wpilib.spark.Spark
    shooter_motor = ctre.CANTalon

    belt_speed = 0
    shooter_speed = 0
    max_shooter_speed = tunable(-0.9)

    def stop(self):
        self.shooter_motor.set(0)
        self.belt_motor.set(0)

        self.done()

    def shoot(self):
        self.engage()

    def force_spin(self):
        self.shooter_speed = self.max_shooter_speed

    def force_feed(self):
        self.belt_speed = -1

    @timed_state(duration=1.0, first=True, next_state='feed_shoot')
    def spinup(self):
        self.shooter_speed = self.max_shooter_speed

    @state
    def feed_shoot(self):
        self.shooter_speed = self.max_shooter_speed
        self.belt_speed = -1

    def execute(self):
        self.belt_motor.set(self.belt_speed)
        self.shooter_motor.set(self.shooter_speed)
