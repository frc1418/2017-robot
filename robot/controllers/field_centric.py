from components import swervedrive, swervemodule
from robotpy_ext.common_drivers import navx
import math


class FieldCentric:
    drive = swervedrive.SwerveDrive

    fl_module = swervemodule.SwerveModule
    fr_module = swervemodule.SwerveModule
    rl_module = swervemodule.SwerveModule
    rr_module = swervemodule.SwerveModule

    navx = navx.AHRS

    def setup(self):
        self.predict_position = False

        self._requested_vectors = {
                'fwd': 0,
                'strafe': 0,
                'rcw': 0
        }

        self._predicted_position = {
                'fwd': 0,
                'strafe': 0,
                'rcw': 0
        }

        self._drive_encoder_zeros = {
                'front_left': 0,
                'rear_right': 0,
                'rear_left': 0,
                'front_right': 0
        }

        self.set_raw_value = False

    def set_fwd(self, fwd):
        self._requested_vectors['fwd'] = fwd

    def set_strafe(self, strafe):
        self._requested_vectors['strafe'] = strafe

    def move(self, fwd, strafe):
        self._requested_vectors['fwd'] = fwd
        self._requested_vectors['strafe'] = strafe

    def execute(self):
        theta = math.radians(360-self.navx.yaw)

        fwdY = self._requested_vectors['fwd'] * math.cos(theta)
        fwdX = (self._requested_vectors['fwd']) * math.sin(theta)  # I think it was because you were negative X in robot.py. X on joysticks is -1 .. 1 left to right
        strafeX = self._requested_vectors['strafe'] * math.cos(theta)
        strafeY = self._requested_vectors['strafe'] * math.sin(theta)

        fwd = fwdY + strafeY
        strafe = fwdX + strafeX

        if not self.set_raw_value:
            self.drive.set_fwd(fwd)
            self.drive.set_strafe(strafe)
        else:
            self.drive.set_raw_fwd(fwd)
            self.drive.set_raw_strafe(strafe)
