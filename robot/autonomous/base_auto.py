from robotpy_ext.autonomous import state, StatefulAutonomous

from components import swervedrive

class VictisAuto(StatefulAutonomous):
    
    drive = swervedrive.SwerveDrive
    
    @state
    def failed(self):
        self.drive.debug()
        self.next_state('done')
        
    @state
    def done(self):
        self.drive.prepare_for_teleop()
        