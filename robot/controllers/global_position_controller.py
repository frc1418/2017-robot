from .pos_controll import XPositionController, YPositionController
from .angle import AngleController


class GlobalPositionController:
    angle_ctrl = AngleController
    xpos_ctrl = XPositionController
    ypos_ctrl = YPositionController
