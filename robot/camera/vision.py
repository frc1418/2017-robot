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
    CV = False
    
    enabled = ntproperty('/camera/enabled', False)
    
    def __init__(self):
        self.nt = NetworkTable.getTable('/camera')
        
        #Cameras
        self.piston_cam = cs.UsbCamera("Piston Cam", 1)
        self.piston_cam.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 160, 120, 10)
        self.piston_cam.setExposureManual(35)
        
        self.light_ring_cam = cs.UsbCamera("Light Ring Cam", 0)
        self.light_ring_cam.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 10)
        
        #Image Processing
        self.cvsink = cs.CvSink("cvsink")
        self.cvsink.setSource(self.light_ring_cam)
        
        self.cvsource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 10)
        
        
        #Streaming Servers
        self.piston_server = cs.MjpegServer("httpserver", 1181)
        self.piston_server.setSource(self.piston_cam)
        
        
        self.ring_stream = cs.MjpegServer("ring server", 1182)
        self.ring_stream.setSource(self.light_ring_cam)
        
        self.cv_stream = cs.MjpegServer("cv stream", 1183)
        self.cv_stream.setSource(self.cvsource)
        
        #Blank mat
        self.img = np.zeros(shape=(320, 240, 3), dtype=np.uint8)
        
        self.processor = ImageProcessor()
    
    def process(self):
        while True:
            
            time, self.img = self.cvsink.grabFrame(self.img)
        
            if time == 0:
                outputStream.notifyError(cvSink.getError())
                continue
            
            if not self.enabled:
                self.light_ring_cam.setExposureManual(35)
                self.nt.putBoolean('processor/gear_target_present', False)
                self.cvsource.putFrame(self.img)
                continue
            else:
                self.light_ring_cam.setExposureManual(3)
            
            self.img = self.processor.process_frame(self.img, time)
                
            self.cvsource.putFrame(self.img)
            pass
    
if __name__ == '__main__':
    main()