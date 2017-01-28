import wpilib
from networktables.networktable import NetworkTable

class GearPicker:
    # The piston that actuates to grab the gear
    picker = wpilib.DoubleSolenoid
    # The piston that actuates the picker up and down
    pivot = wpilib.DoubleSolenoid

    def setup(self):
        self.sd = NetworkTable.getTable('SmartDashboard')

        self._picker_state = self.picker.get()
        self._pivot_state = self.pivot.get()

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

    def pivot_down(self):
        """Pivot picker arm down."""
        self._pivot_state = 2

    def update_sd(self, name):
        """Put refreshed values to SmartDashboard."""
        pass

    def execute(self):
        """Repeating code."""
        self.picker.set(self._picker_state)
        self.pivot.set(self._pivot_state)
