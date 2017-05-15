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

    intake_motor = wpilib.VictorSP

    def setup(self):
        self.sd = NetworkTable.getTable('SmartDashboard')

        self._picker_state = 1
        self._pivot_state = 1

        self.intake_on = False

    def on_enable(self):
        pass

    def actuate_picker(self):
        """
        Switch picker state.
        """
        if self._picker_state == 1:
            self._picker_state = 2
        else:
            self._picker_state = 1

    def pivot_up(self):
        """
        Pivot picker arm up.
        """
        self._pivot_state = 1
        self.intake_on = True

    def pivot_down(self):
        """
        Pivot picker arm down.
        """
        self._pivot_state = 2
        self.intake_on = False

    def update_sd(self):
        """
        Put refreshed values to SmartDashboard.
        """
        self.sd.putBoolean('picker/intake_on', self.intake_on)
        self.sd.putBoolean('picker/pivot_state', self.pivot.get() == 1)

    def execute(self):
        """
        Repeating code.
        """
        self.update_sd()
        self.picker.set(self._picker_state)
        self.pivot.set(self._pivot_state)

        if self._pivot_state == 1 and self.intake_on:
            self.intake_motor.set(1)
        else:
            self.intake_motor.set(0)
