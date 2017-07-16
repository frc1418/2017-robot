from magicbot.state_machine import state, AutonomousStateMachine

from components import swervedrive


class VictisAuto(AutonomousStateMachine):
    """
    This class is used to flush the drive system at the end of autonomous.
    Doing this reduces potential errors with variable errors carrying over.
    """
    drive = swervedrive.SwerveDrive

    @state
    def failed(self):
        """
        This state should be called when an auto mode has failed.
        """
        self.drive.debug(debug_modules=True) #Prints some debugging info
        self.next_state('finish')

    @state
    def finish(self):
        self.drive.flush()
        self.done()
