from components import swervedrive
from magicbot.state_machine import state, AutonomousStateMachine


class VictisAuto(AutonomousStateMachine):
    drive = swervedrive.SwerveDrive

    @state
    def failed(self):
        self.drive.debug(debug_modules=True)
        self.next_state('finish')

    @state
    def finish(self):
        self.drive.flush()
        self.done()
