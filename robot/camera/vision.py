"""
To run this without the robot (must have cscore installed on Linux):

    python3 -m cscore vision.py:main
"""

import cscore as cs
import cv2
import numpy as np

from image_processor import ImageProcessor

from networktables import NetworkTable
from networktables.util import ntproperty


def main():
    vision = VictisVision()
    vision.process()


class VictisVision:
    STREAM_CV = False

    cv_enabled = ntproperty('/camera/control/cv_enabled', False)
    dark_exposure = ntproperty('/camera/control/dark_exposure', 3)

    test = ntproperty('/camera/control/test', 0)

    def __init__(self):
        self.nt = NetworkTable.getTable('/camera')

        # Cameras
        self.piston_cam = cs.UsbCamera('Piston Cam', 1)
        self.piston_cam.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 20) #160 vs. 120

        self.piston_cam.setExposureAuto()
        self.piston_cam.getProperty('backlight_compensation').set(5)

        self.light_ring_cam = cs.UsbCamera('Light Ring Cam', 0)
        self.light_ring_cam.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 20)

        # This only seems to affect automatic exposure mode
        # -> higher value means it lets more light in when facing a big light
        self.light_ring_cam.getProperty('backlight_compensation').set(5)

        # Image Processing
        self.cvsink = cs.CvSink('cvsink')
        self.cvsink.setSource(self.light_ring_cam)

        self.cvsource = cs.CvSource('cvsource', cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 20)

        # Streaming Servers

        self.piston_server = cs.MjpegServer('httpserver', 1181)
        self.piston_server.setSource(self.piston_cam)

        if self.STREAM_CV:
            self.cv_stream = cs.MjpegServer('cv stream', 1183)
            self.cv_stream.setSource(self.cvsource)

        # Blank mat
        self.img = np.zeros(shape=(320, 240, 3), dtype=np.uint8)

        self.processor = ImageProcessor()

    def process(self):

        exposure = None

        while True:
            if self.cv_enabled:

                if exposure != 'dark':
                    self.light_ring_cam.setExposureManual(int(self.dark_exposure))
                    exposure = 'dark'

                time, self.img = self.cvsink.grabFrame(self.img)

                if time == 0:
                    self.cvsource.notifyError(self.cvsink.getError())
                    continue

                self.img = self.processor.process_frame(self.img, time)
            else:
                if exposure != 'auto':
                    self.light_ring_cam.setExposureAuto()
                    exposure = 'auto'

                self.nt.putBoolean('processor/gear_target_present', False)

            if self.STREAM_CV:
                self.cvsource.putFrame(self.img)


if __name__ == '__main__':
    main()
