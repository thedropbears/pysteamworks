import numpy as np
import cv2
import heapq

AREA_THRESHOLD = 0.0005

def loop():
    import cscore as cs
    from networktables import NetworkTables
    from time import sleep

    nt = NetworkTables.getTable("/components/vision")

    width = 320
    height = 240
    fps = 25
    videomode = cs.VideoMode.PixelFormat.kMJPEG, width, height, fps

    front_camera = cs.UsbCamera("frontcam", 0)
    front_camera.setVideoMode(*videomode)

    front_cam_server = cs.MjpegServer("frontcamserver", 8082)
    front_cam_server.setSource(front_camera)

    back_camera = cs.UsbCamera("backcam", 1)
    back_camera.setVideoMode(*videomode)

    back_cam_server = cs.MjpegServer("backcamserver", 8081)
    back_cam_server.setSource(back_camera)

    cvsink = cs.CvSink("cvsink")
    cvsink.setSource(front_camera)

    cvSource = cs.CvSource("cvsource", *videomode)
    cvmjpegServer = cs.MjpegServer("cvhttpserver", 8083)
    cvmjpegServer.setSource(cvSource)

    # Set the initial exposure of the front camera.
    # It may take a while for the exposure setting to have an effect.
    front_camera.setExposureManual(0)

    # Images are big. Preallocate an array to fill the image with.
    frame = np.zeros(shape=(height, width, 3), dtype=np.uint8)

    while True:
        if nt.getBoolean("vision_mode", True):
            front_camera.setExposureManual(0)
            time, frame = cvsink.grabFrame(frame)
            if time == 0:
                # We got an error; report it through the output stream.
                cvSource.notifyError(cvsink.getError())
            else:
                x, img, num_targets = find_target(frame)
                if num_targets > 0:
                    nt.putNumber("x", x)
                    nt.putNumber("time", time)
                    nt.putNumber("num_targets", num_targets)
                cvSource.putFrame(img)
        else:
            # Let the driver use the front camera to see.
            front_camera.setExposureAuto()
            # Avoid pegging the CPU.
            sleep(1)


def find_target(img, lower=np.array([110//2, 40*255//100, 5*255//100]), upper=np.array([180//2, 100*255//100, 100*255//100])):
    """Given an image and thresholds, find the centre of mass of the target.

    All arguments must be np.arrays, and lower and upper must be a 3-array.
    """

    #Converting from RGB to HSV.
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #Making the mask.
    mask = cv2.inRange(hsv, lower, upper)

    #Combining the mask with the frame
    res = cv2.bitwise_and(img, img, mask=mask)

    height, width = mask.shape
    # Get the information for the contours
    _, contours, __ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # sort the contours into a list
    areas = []
    for idx, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area/(height*width) > AREA_THRESHOLD:
            heapq.heappush(areas, (cv2.contourArea(contour), idx))

    areas = heapq.nlargest(2, areas)
    x_coord = 0
    for _, idx in areas:
        contour = contours[idx]
        moments = cv2.moments(contour)
        x_coord += moments['m10']/moments['m00'] / len(areas)
        cv2.drawContours(res, (contour, ), -1, (255, 0, 0), 1)
    if len(areas) > 0:
        cv2.line(res, (int(x_coord), 60), (int(x_coord), 180), (255,255,0), thickness=2, lineType=8, shift=0)
    pos = 2 * x_coord / width - 1
    return pos, res, len(areas)

# Allow easy testing of captured sample images
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Test vision system from command line.')
    parser.add_argument('--showfile', help='display a specific file with a bounding box drawn around it',
            type=str, default=None)
    args = parser.parse_args()
    if args.showfile:
        image = cv2.imread(args.showfile, cv2.IMREAD_COLOR)
        x, image, num_targets = find_target(image)
        cv2.imshow('image', image)
    while cv2.getWindowHandle('image') >= 0:
        if cv2.waitKey(50) != -1:
            break

