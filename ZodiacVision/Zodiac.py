import cv2
import math
import numpy as np


# Team 1816's 2020 Vision Processing Code
# Used with OpenCV and Gstreamer
def make_half_hex_shape():
    pts = []
    for ang in [0, 60, 120, 180]:
        ang_r = math.radians(ang)
        x1 = int(100.0 * math.cos(ang_r) + 100.5)
        y1 = int(100.0 * math.sin(ang_r) + 100.5)
        pts.append([x1, y1])

    for ang in [180, 120, 60, 0]:
        ang_r = math.radians(ang)
        x1 = int(92.0 * math.cos(ang_r) + 100.5)
        y1 = int(92.0 * math.sin(ang_r) + 100.5)
        pts.append([x1, y1])
    shape_np = np.array(pts, np.int32)
    shape_np = np.reshape(shape_np, (-1, 1, 2))
    return shape_np


def preProcess(image):
    # lower_color = (60, 106, 150)
    # upper_color = (130, 255, 255)
    lower_color = (47, 72, 150)
    upper_color = (93, 255, 255)
    h, w, _ = image.shape
    # image = image[0:int(h / 2), 0:w]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    return mask


def findTarget(image, shape):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    hexes = []
    largest = None
    firstLoop = True
    for cont in contours:
        if firstLoop:
            largest = cont
            firstLoop = False
            continue
        if cont.size > largest.size:
            largest = cont

    if largest is not None:
        rect = cv2.boundingRect(largest)
    # only process larger areas with at least 5 points in the contour
    if True or (len(largest) > 4 and rect[2] > 40 and rect[3] > 40):
        match = cv2.matchShapes(largest, shape, cv2.CONTOURS_MATCH_I2, 0.0)
        print(match)
        if match < 10:
            return largest
    return -1


def postProcess(image, target):
    if target is -1:
        return image
    drawnimage = cv2.drawContours(image, [target], -1, (0, 255, 255), 2)
    return drawnimage


if __name__ == '__main__':
    fps = 30.
    frame_width = 640
    frame_height = 480
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    gst_str_rtp = "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=127.0.0.1 port=5000"

    # Check if cap is open
    # Create videowriter as a SHM sink
    out = cv2.VideoWriter(gst_str_rtp, 0, fps, (frame_width, frame_height), True)

    hexes = make_half_hex_shape()
    while 1:
        _, frame = cap.read()
        preProcessImage = preProcess(frame)
        cv2.imshow("PreProcessed Image", preProcessImage)
        target = findTarget(preProcessImage, hexes)
        postProcessImage = postProcess(frame, target)
        cv2.imshow("PostProcessed Image", postProcessImage)
        cv2.waitKey(1)
        # Write to SHM
        out.write(frame)
