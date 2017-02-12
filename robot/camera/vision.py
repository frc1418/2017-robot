import cscore as cs
import cv2
import numpy as np

from image_processor import ImageProcessor

from networktables import NetworkTable

def main():
    vision = VictisVision()
    vision.process()
    
class VictisVision:
    
    enabled = ntproperty('/camera/enabled', False)
    
    def __init__(self):
        NetworkTable.setIPAddress('localhost')
        NetworkTable.setClientMode()
        NetworkTable.initialize()
        
        self.nt = NetworkTable.getTable('/camera')
        
        #Cameras
        self.piston_cam = cs.CameraPort("Piston Cam", 0)
        self.piston_cam.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 30)
        
        self.light_ring_cam = cs.CameraPort("Light Ring Cam", 1)
        self.light_ring_cam.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 30)
        
        #Image Processing
        self.cvsink = cs.CvSink("cvsink")
        self.cvsink.setSource(self.light_ring_cam)
        
        self.cvsource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 30)
        
        #Streaming Servers
        self.piston_server = cs.MjpegServer("httpserver", 8081)
        self.piston_server.setSource(self.piston_cam)
        
        self.ring_stream = cs.MjpegServer("ring server", 8082)
        self.ring_stream.setSource(self.light_ring_cam)
        
        self.cv_stream = cs.MjpegServer("cv stream", 8083)
        self.cv_stream.setSource(self.cvsource)
        
        #Blank mat
        self.img = np.zeros(shape=(320, 240, 3), dtype=np.uint8)
        
        self.processor = ImageProcessor()
    
    def process(self):
        while True:
            time, self.img = cvsink.grabFrame(self.img)
        
            if time == 0:
                outputStream.notifyError(cvSink.getError())
                continue
            
            if not self.enabled:
                self.nt.putBoolean('processor/gear_target_present', False)
                self.cvSource.putFrame(img)
                continue
            
            self.img = self.processor.process_frame(self.img, time)
                
            self.cvSource.putFrame(self.img)
    
if __name__ == '__main__':
    main()