from magicbot import StateMachine, timed_state, state
import wpilib
import ctre
from magicbot.magic_tunable import tunable

class Shooter(StateMachine):
    belt_motor = wpilib.spark.Spark
    shooter_motor = ctre.CANTalon
    
    shooter_speed = tunable(-0.9)
        
    def stop(self):
        self.shooter_motor.set(0)
        self.belt_motor.set(0)
        
        self.done()
    
    def shoot(self):
        self.engage()
    
    @timed_state(duration=1.0, first=True, next_state='feed_shoot')
    def spinup(self):
        self.shooter_motor.set(self.shooter_speed)
    
    @state  
    def feed_shoot(self):
        self.shooter_motor.set(self.shooter_speed)
        self.belt_motor.set(-1)
        