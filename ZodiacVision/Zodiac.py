import cv2
import math
import numpy as np
import networktables as nt
import argparse

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
        x1 = int(80.0 * math.cos(ang_r) + 100.5)
        y1 = int(80.0 * math.sin(ang_r) + 100.5)
        pts.append([x1, y1])
    shape_np = np.array(pts, np.int32)
    shape_np = np.reshape(shape_np, (-1, 1, 2))
    return shape_np


def preProcess(image):
    # lower_color = (60, 106, 150)
    # upper_color = (130, 255, 255)
    lower_color = (55, 70, 143)
    upper_color = (94, 255, 255)
    h, w, _ = image.shape
    # image = image[0:int(h / 2), 0:w]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    return mask


def findTarget(image, shape, nt_table):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    hexes = []
    firstLoop = True
    if len(contours) != 0:
        c = max(contours, key=cv2.contourArea)
        rect = cv2.boundingRect(c)
    else:
        return -1
    print(len(contours))

    # only process larger areas with at least 5 points in the contour
    if len(c) > 20 and rect[2] > 10 and rect[3] > 10:
        cv2.drawContours(frame, [c], -1, (0, 255, 255), 2)
        match = cv2.matchShapes(c, shape, cv2.CONTOURS_MATCH_I2, 0.0)
        print(match)
        if match < 5:
            cx = rect[0] + (rect[2] * .5)
            cy = rect[1] + (rect[3] * .5)
            nt_table.putNumber('center_x', cx)
            nt_table.putNumber('center_y', cy)
            return c
    clearNetworkTables(nt_table)
    return -1


def postProcess(image, target):
    if target is -1:
        return image
    drawnimage = cv2.drawContours(image, hexes, -1, (0, 255, 255), 2)
    return drawnimage


def clearNetworkTables(table):
    table.putNumber('center_x', -1)
    table.putNumber('center_y', -1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--view", help="enable windows")
    args = parser.parse_args()

    fps = 30
    frame_width = 640
    frame_height = 480
    cap = cv2.VideoCapture("Left_Frame.avi")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    
    gst_str_rtp = "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=127.0.0.1 port=5000"
    # Check if cap is open
    # Create videowriter as a SHM sink
    out = cv2.VideoWriter(gst_str_rtp, 0, fps, (frame_width, frame_height), True)
    nt.NetworkTables.initialize(server='10.18.16.2')
    table = nt.NetworkTables.getTable('SmartDashboard')
    if table:
        print('table OK')
    clearNetworkTables(table)
    hexes = make_half_hex_shape()
    while 1:
        _, frame = cap.read()
        preProcessImage = preProcess(frame)
        if args.view:
            cv2.imshow("PreProcessed Image", preProcessImage)

        target = findTarget(preProcessImage, hexes, table)
        if args.view:
            postProcessImage = postProcess(frame, target)
            cv2.imshow("PostProcessed Image", postProcessImage)
            cv2.waitKey(1)
        # Write to SHM
        out.write(frame)
