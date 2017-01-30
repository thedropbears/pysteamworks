import numpy as np
import cv2
import argparse
import cscore as cs

class Vision:
    width = 320
    height = 240

    def __init__(self):
        self.camera = cs.UsbCamera("usbcam", 0)
        self.camera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, Vision.width, Vision.height, 20)
    
        self.camMjpegServer = cs.MjpegServer("httpserver", 8082)
        self.camMjpegServer.setSource(self.camera)

        self.cvsink = cs.CvSink("cvsink")
        self.cvsink.setSource(self.camera)
        
        self.cvSource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, Vision.width, Vision.height, 30)
        self.cvmjpegServer = cs.MjpegServer("cvhttpserver", 8083)
        self.cvmjpegServer.setSource(self.cvSource)

        #Setting the exposure.
        self.camera.setExposureManual(10)

    def setup(self):
        """Run just after createObjects.
        Useful if you want to run something after just once after the
        robot code is started, that depends on injected variables"""
        pass

    def on_enable(self):
        """Run every time the robot transitions to being enabled"""
        pass

    def on_disable(self):
        """Run every time the robot transitions to being disabled"""
        pass

    def execute(self):
        frame = np.zeros(shape=(Vision.height, Vision.width, 3), dtype=np.uint8)
        time, frame = self.cvsink.grabFrame(frame)
        if time == 0:
            frame = np.zeros(shape=(Vision.height, Vision.width, 3), dtype=np.uint8)
        else:
            self.x, frame = self.find_target(frame)
            cv2.line(frame, (int((self.x+1)*Vision.width/2), 60), (int((self.x+1)*Vision.width/2), 180), (255,255,0), thickness=2, lineType=8, shift=0)
        self.cvSource.putFrame(frame)

    def find_target(self, img):

        #Converting from RGB to HSV.
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
        #Setting the tolerances for the mask.
        upper_green = np.array([150/2, 200, 200])
        lower_green = np.array([110/2, 40, 70])
    
        #Making the mask.
        mask = cv2.inRange(hsv, lower_green, upper_green)
    
        #Combining the mask with the frame
        res = cv2.bitwise_and(img, img, mask=mask)

        x = range(0, mask.shape[1])
        y = range(0, mask.shape[0])

        (X,Y) = np.meshgrid(x,y)

        try:
            x_coord = int((X*mask).sum()/mask.sum().astype("float"))
        except ValueError:
            x_coord = 0

        width = mask.shape[1]
        pos = 2 * x_coord / float(width) - 1
        return pos, res
