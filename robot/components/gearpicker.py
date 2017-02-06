import wpilib
from networktables.networktable import NetworkTable
from components import gimbal

class GearPicker:
    # The piston that actuates to grab the gear
    picker = wpilib.DoubleSolenoid
    # The piston that actuates the picker up and down
    pivot = wpilib.DoubleSolenoid
    
    intake_motor = wpilib.VictorSP
    gimbal = gimbal.Gimbal

    

    def setup(self):
        self.sd = NetworkTable.getTable('SmartDashboard')

        self._picker_state = self.picker.get()
        self._pivot_state = self.pivot.get()

        self.intake_on = False

    def on_enable(self):
        pass

    def actuate_picker(self):
        """Switch picker state."""
        if self._picker_state == 1:
            self._picker_state = 2
        else:
            self._picker_state = 1

    def pivot_up(self):
        """Pivot picker arm up."""
        self._pivot_state = 1
        
        self.gimbal.yaw = 0.0
        self.gimbal.pitch = 0.4
        #self.sd.putNumber('/camera/gimbal/yaw', 0.0)
        #self.sd.putNumber('/camera/gimbal/pitch', 0.4)
        
        if self.intake_on:
            self.intake_motor.set(1)

    def pivot_down(self):
        """Pivot picker arm down."""
        self._pivot_state = 2
        
        self.gimbal.yaw = 0.15
        self.gimbal.pitch = 0.7
        #self.sd.putNumber('/camera/gimbal/yaw', 0.15)
        #self.sd.putNumber('/camera/gimbal/pitch', 0.7)
        
        self.intake_motor.set(0)

    def update_sd(self, name):
        """Put refreshed values to SmartDashboard."""
        pass

    def execute(self):
        """Repeating code."""
        self.picker.set(self._picker_state)
        self.pivot.set(self._pivot_state)
