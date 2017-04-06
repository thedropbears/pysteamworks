import numpy as np
import cv2
import heapq


def loop():  # pragma: no cover
    import cscore as cs
    from networktables import NetworkTables
    from time import sleep, monotonic as now

    nt = NetworkTables.getTable("/components/vision")

    width = 320
    height = 240
    fps = 25
    videomode = cs.VideoMode.PixelFormat.kMJPEG, width, height, fps

    #front_cam_id = "/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_5FC7BADF-video-index0"
    front_cam_id = "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_332E8C9F-video-index0"
    # winch_cam_id = "/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_5FC7BADF-video-index0"

    # winch_camera = cs.UsbCamera("winchcam", winch_cam_id)
    # winch_camera.setVideoMode(*videomode)

    # winch_cam_server = cs.MjpegServer("winchcamserver", 5802)
    # winch_cam_server.setSource(winch_camera)

    front_camera = cs.UsbCamera("frontcam", front_cam_id)
    front_camera.setVideoMode(*videomode)

    front_cam_server = cs.MjpegServer("frontcamserver", 5803)
    front_cam_server.setSource(front_camera)

    cvsink = cs.CvSink("cvsink")
    cvsink.setSource(front_camera)

    cvSource = cs.CvSource("cvsource", *videomode)
    cvmjpegServer = cs.MjpegServer("cvhttpserver", 5804)
    cvmjpegServer.setSource(cvSource)

    # Images are big. Preallocate an array to fill the image with.
    frame = np.zeros(shape=(height, width, 3), dtype=np.uint8)

    # Set the exposure to something bogus, in an attempt to work around a kernel bug when restarting the vision code?
    front_camera.setExposureAuto()
    sleep(1)
    cvsink.grabFrame(frame)

    # Set the exposure of the front camera to something usable.
    front_camera.setExposureManual(1)
    sleep(1)
    front_camera.setExposureManual(0)

    while True:
        time, frame = cvsink.grabFrame(frame)
        if time == 0:
            # We got an error; report it through the output stream.
            cvSource.notifyError(cvsink.getError())
        else:
            t = now()
            x, img, num_targets, target_sep = find_target(frame)
            if num_targets > 0:
                nt.putNumber("x", x)
                nt.putNumber("time", t)
                nt.putNumber("num_targets", num_targets)
                nt.putNumber("target_sep", target_sep)
                nt.putNumber("dt", now() - t)
                NetworkTables.flush()
            cvSource.putFrame(img)


def find_target(img, lower=np.array([110//2, 10*255//100, 20*255//100]), upper=np.array([180//2, 100*255//100, 100*255//100]), area_threshold=0.025 ** 2):
    """Given an image and thresholds, find the centre of mass of the target.
    All arguments must be np.arrays, except for area_threshold, and lower and upper must be a 3-array.
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
        if area/(height*width) > area_threshold:
            heapq.heappush(areas, (cv2.contourArea(contour), idx))

    areas = heapq.nlargest(2, areas)
    areas_x = []
    x_coord = 0
    for _, idx in areas:
        contour = contours[idx]
        moments = cv2.moments(contour)
        x_coord += moments['m10']/moments['m00'] / len(areas)
        areas_x.append(moments['m10']/moments['m00'])
        cv2.drawContours(res, (contour, ), -1, (255, 0, 0), 1)
    if len(areas) > 0:
        cv2.line(res, (int(x_coord), 60), (int(x_coord), 180), (255,255,0), thickness=2, lineType=8, shift=0)
    target_sep = 0
    if len(areas_x) > 1:
        # target sep returned as a % of image width, not in vision coordinates
        target_sep = abs(areas_x[0]-areas_x[1]) / width
    pos = 2 * x_coord / width - 1
    return pos, res, len(areas), target_sep

# Allow easy testing of captured sample images
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Test vision system from command line.')
    parser.add_argument('--showfile', help='display a specific file with a bounding box drawn around it',
            type=str, default=None)
    args = parser.parse_args()
    if args.showfile:
        image = cv2.imread(args.showfile, cv2.IMREAD_COLOR)
        x, image, num_targets, target_sep = find_target(image)
        cv2.imshow('image', image)
        while True:
            if cv2.waitKey(50) != -1:
                break
