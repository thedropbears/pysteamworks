import multiprocessing.sharedctypes

import numpy as np
import cv2
import hal
import wpilib

from magicbot import tunable


class Vision:
    width = 320
    height = 240
    fps = 25
    k = tunable(0.5)

    def __init__(self):
        self._data_array = multiprocessing.sharedctypes.RawArray("d", [0.0])

        if not hal.isSimulation():
            self._process_run_event = multiprocessing.Event()
            self._process_run_event.set()
            self._process = multiprocessing.Process(target=vision_loop, args=(self._data_array, self._process_run_event))
            self._process.start()

        self.smoothed_x = 0.0

        # Register with Resource so teardown works
        wpilib.Resource._add_global_resource(self)

    def free(self):
        if not hal.isSimulation():
            self._process_run_event.clear()
            self._process.join(0.1)
            self._process.terminate()

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


def vision_loop(data_array, run_event):
    import cscore as cs

    videomode = cs.VideoMode.PixelFormat.kMJPEG, Vision.width, Vision.height, Vision.fps

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
    camera.setExposureManual(10)

    while run_event.is_set():
        frame = np.zeros(shape=(Vision.height, Vision.width, 3), dtype=np.uint8)
        time, frame = cvsink.grabFrame(frame)
        if time == 0:
            frame = np.zeros(shape=(Vision.height, Vision.width, 3), dtype=np.uint8)
        else:
            x, frame = find_target(frame)
            if x is not None:
                cv2.line(frame, (int((x+1)*Vision.width/2), 60), (int((x+1)*Vision.width/2), 180), (255,255,0), thickness=2, lineType=8, shift=0)
                data_array[0] = x
        cvSource.putFrame(frame)


def find_target(img):
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
        return None, res

    width = mask.shape[1]
    pos = 2 * x_coord / float(width) - 1
    return pos, res
