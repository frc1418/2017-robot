from wpilib import AnalogInput

class REVAnalogPressureSensor():
    
    VOLTAGE_IN = 3.2
    
    def __init__(self, channel):
        self.pressure = AnalogInput(channel)
        
    def getPressure(self):
        v = max(self.pressure.getVoltage(), 0.00001)
        return (250*(v/self.VOLTAGE_IN))-25