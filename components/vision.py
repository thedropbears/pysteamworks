import multiprocessing.sharedctypes

import numpy as np
import cv2
import hal

from magicbot import tunable

MIN_MASKED_RATIO = 0.1


class Vision:
    k = tunable(0.5)

    def __init__(self):
        self._data_array = multiprocessing.sharedctypes.RawArray("d", [0.0, 0.0])

        if not hal.isSimulation():
            self._process = multiprocessing.Process(target=vision_loop, args=(self._data_array,), daemon=True)
            self._process.start()

        self.smoothed_x = 0.0

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
        """Run at the end of the control loop"""
        self.smoothed_x = (1 - self.k) * self.x + self.k * self.smoothed_x

    @property
    def x(self):
        return self._data_array[0]

    @property
    def time(self):
        return self._data_array[1]


def vision_loop(data_array):
    import cscore as cs

    width = 320
    height = 240
    fps = 25
    videomode = cs.VideoMode.PixelFormat.kMJPEG, width, height, fps

    camera = cs.UsbCamera("usbcam", 0)
    camera.setVideoMode(*videomode)

    camMjpegServer = cs.MjpegServer("httpserver", 8082)
    camMjpegServer.setSource(camera)

    cvsink = cs.CvSink("cvsink")
    cvsink.setSource(camera)

    cvSource = cs.CvSource("cvsource", *videomode)
    cvmjpegServer = cs.MjpegServer("cvhttpserver", 8083)
    cvmjpegServer.setSource(cvSource)

    #Setting the exposure.
    camera.setExposureManual(0)

    # Images are big. Preallocate an array to fill the image with.
    frame = np.zeros(shape=(height, width, 3), dtype=np.uint8)

    while True:
        time, frame = cvsink.grabFrame(frame)
        if time == 0:
            # We got an error; report it through the output stream.
            cvSource.notifyError(cvsink.getError())
        else:
            x, img, masked_ratio = find_target(frame)
            print(masked_ratio)
            if masked_ratio > MIN_MASKED_RATIO:
                loc = int((x+1) * width // 2)
                cv2.line(img, (loc, 60), (loc, 180), (255, 255, 0), thickness=2, lineType=8, shift=0)
                data_array[0] = x
                data_array[1] = time
            cvSource.putFrame(img)

def find_target(img, lower=np.array([110/2, 50, 50]), upper=np.array([155/2, 255, 255])):

    #Converting from RGB to HSV.
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #Making the mask.
    mask = cv2.inRange(hsv, lower, upper)

    #Combining the mask with the frame
    res = cv2.bitwise_and(img, img, mask=mask)

    height, width = mask.shape
    masked_pixels = mask.sum()
    masked_ratio = masked_pixels / (height*width)

    if masked_ratio > MIN_MASKED_RATIO:
        X, Y = np.meshgrid(range(0, width), range(0, height))
        x_coord = int((X * mask).sum() / masked_pixels)
    else:
        return None, res, masked_ratio

    pos = 2 * x_coord / width - 1
    return pos, res, masked_ratio
