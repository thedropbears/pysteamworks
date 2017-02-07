import multiprocessing.sharedctypes

import numpy as np
import cv2
import hal
import wpilib

from magicbot import tunable


class Vision:
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
    camera.setExposureManual(10)

    # Images are big. Preallocate an array to fill the image with.
    frame = np.zeros(shape=(height, width, 3), dtype=np.uint8)

    while run_event.is_set():
        time, frame = cvsink.grabFrame(frame)
        if time == 0:
            # We got an error; report it through the output stream.
            cvSource.notifyError(cvsink.getError())
        else:
            x, img = find_target(frame)
            if x is not None:
                loc = int((x+1) * width // 2)
                cv2.line(frame, (loc, 60), (loc, 180), (255, 255, 0), thickness=2, lineType=8, shift=0)
                data_array[0] = x
            cvSource.putFrame(img)


def find_target(img, lower=np.array([110/2, 40, 70]), upper=np.array([150/2, 200, 200])):
    #Converting from RGB to HSV.
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #Making the mask.
    mask = cv2.inRange(hsv, lower, upper)

    #Combining the mask with the frame
    res = cv2.bitwise_and(img, img, mask=mask)

    height, width = mask.shape
    X, Y = np.meshgrid(range(0, width), range(0, height))

    try:
        x_coord = int((X * mask).sum() / mask.sum())
    except ValueError:
        return None, res

    pos = 2 * x_coord / width - 1
    return pos, res
