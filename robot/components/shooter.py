from magicbot import StateMachine, timed_state, state
import wpilib
import ctre

class Shooter(StateMachine):
    belt_motor = wpilib.spark.Spark
    shooter_motor = ctre.CANTalon

    def stop(self):
        self.shooter_motor.set(0)
        self.belt_motor.set(0)

        self.done()

    def shoot(self):
        self.engage()

    @timed_state(duration=2.0, first=True, next_state='feed_shoot')
    def spinup(self):
        self.shooter_motor.set(-1)

    @state
    def feed_shoot(self):
        self.shooter_motor.set(-1)
        self.belt_motor.set(1)
