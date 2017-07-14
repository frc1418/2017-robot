import math


class BasePIDComponent:
    """
    Base for a component that has a PIDController controlling its output.

    Readonly variables for subclasses:

    * self.enabled: set to True if enabled (only read this)
    * self.rate: set to the calculated rate if enabled, None otherwise

    variables that subclasses should set:

    * self.setpoint: must be set each period to enable the controller

    TODO: add a 'settle time' output variable
    """

    ramp = 0.1

    MIN_RAW_OUTPUT = -1.0
    MAX_RAW_OUTPUT = 1.0

    MIN_ABS_OUTPUT = 0.0
    MAX_ABS_OUTPUT = 1.0

    def __init__(self, pid_input, table_name):
        self.enabled = False
        self.rate = None

        self.err = 0

        self._has_setpoint = False
        self._setpoint = None

        self._pid_input = pid_input

        self._abs_min = 0.0
        self._abs_max = 1.0

    @property
    def setpoint(self):
        return self._setpoint

    @setpoint.setter
    def setpoint(self, value):
        self._setpoint = value
        self._has_setpoint = True

    def compute_error(self, setpoint, pid_input):
        return setpoint - pid_input

    def pidWrite(self, output):
        self.rate = output

    def set_abs_output_range(self, min_, max_):
        self._abs_min = min_
        self._abs_max = max_

    def execute(self):
        if self._has_setpoint:

            if not self.enabled:
                self.enabled = True
                self._last_output = 0
                self._last_err = 0
                self.err = 0

            # homemade PID
            err = self.compute_error(self._setpoint, self._pid_input())
            if abs(err) < self.kIzone:
                self.err = 0
            else:
                self.err += err

            output = err * self.kP + self.kI * self.err + self.kD * (err - self._last_err)
            output = min(self.MAX_RAW_OUTPUT, max(output, self.MIN_RAW_OUTPUT))

            scaled_output = ((abs(output) - self.MIN_ABS_OUTPUT) / (self.MAX_ABS_OUTPUT - self.MIN_ABS_OUTPUT) * (self._abs_max - self._abs_min)) + self._abs_min
            output = math.copysign(scaled_output, output)

            self._last_output = output
            self._last_err = err

            self.pidWrite(output)

        else:
            if self.enabled:
                self.enabled = False
                self.rate = None

        self._has_setpoint = False
