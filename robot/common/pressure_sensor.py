from wpilib import AnalogInput

class REVAnalogPressureSensor():
    
    VOLTAGE_IN = 3.2
    
    def __init__(self, channel):
        self.pressure = AnalogInput(channel)
        
    def getPressure(self):
        return (250*(self.pressure.getVoltage()/self.VOLTAGE_IN))-25