import numpy as np
import cv2


MIN_MASKED_RATIO = 0.1


def loop():
    import cscore as cs
    from networktables import NetworkTables

    nt = NetworkTables.getTable("/components/vision")

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
            if masked_ratio > MIN_MASKED_RATIO:
                loc = int((x+1) * width // 2)
                cv2.line(img, (loc, 60), (loc, 180), (255, 255, 0), thickness=2, lineType=8, shift=0)
                nt.putNumber("x", x)
                nt.putNumber("time", time)
            cvSource.putFrame(img)


def find_target(img, lower=np.array([110/2, 200, 75]), upper=np.array([155/2, 255, 255])):
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
