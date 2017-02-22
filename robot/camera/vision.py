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
    
    test = ntproperty('/camera/control/test', 0)
    
    #show_piston =  ntproperty('/camera/control/show_piston', True, True)
    #show_main =  ntproperty('/camera/control/show_main', False, True)
    
    def __init__(self):
        self.nt = NetworkTable.getTable('/camera')
        
        #Cameras
        self.piston_cam = cs.UsbCamera("Piston Cam", 1)
        self.piston_cam.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 20) #160 vs. 120
        self.piston_cam.setExposureManual(35)
        self.piston_cam.setBrightness(65)
        test = self.piston_cam.getProperty("exposure_absolute")
        print(self.piston_cam.getProperty("exposure_absolute"))
        
        self.light_ring_cam = cs.UsbCamera("Light Ring Cam", 0)
        self.light_ring_cam.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 20)
        
        #Image Processing
        self.cvsink = cs.CvSink("cvsink")
        self.cvsink.setSource(self.light_ring_cam)
        
        self.cvsource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 20)
        
        #Streaming Servers
        
        self.piston_server = cs.MjpegServer("httpserver", 1181)
        self.piston_server.setSource(self.piston_cam)
        
        
        self.ring_stream = cs.MjpegServer("ring server", 1182)
        self.ring_stream.setSource(self.light_ring_cam)
        
        if self.STREAM_CV:
            self.cv_stream = cs.MjpegServer("cv stream", 1183)
            self.cv_stream.setSource(self.cvsource)
        
        #Blank mat
        self.img = np.zeros(shape=(320, 240, 3), dtype=np.uint8)
        
        self.processor = ImageProcessor()
    
    def process(self):
        while True:
            #self.piston_cam.setExposureManual(35)
            #self.piston_cam.setBrightness(74)
            
            if self.cv_enabled:
                #if self.getProperty('')
                self.light_ring_cam.setExposureManual(3)
                
                time, self.img = self.cvsink.grabFrame(self.img)
        
                if time == 0:
                    outputStream.notifyError(cvSink.getError())
                    continue
                
                self.img = self.processor.process_frame(self.img, time) 
            else:
                #self.light_ring_cam.setExposureManual(35)
                #self.light_ring_cam.setBrightness(52)
                
                
                
                self.nt.putBoolean('processor/gear_target_present', False)
                
            if self.STREAM_CV:
                self.cvsource.putFrame(self.img)
    
if __name__ == '__main__':
    main()