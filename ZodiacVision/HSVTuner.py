import cv2
#import pyzed.sl as sl
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
    # lower_color = (edge_params['min_h'], edge_params['min_s'], edge_params['min_v'])
    # upper_color = (edge_params['max_h'], edge_params['max_s'], edge_params['max_v'])
    # image = image[0:int(h / 2), 0:w]
    # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # mask = cv2.inRange(hsv, lower_color, upper_color)

    return image
def callback(x):
    pass



if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    cv2.namedWindow('image')
    cv2.createTrackbar('lowH', 'image', 0, 179, callback)
    cv2.createTrackbar('highH', 'image', 179, 179, callback)

    cv2.createTrackbar('lowS', 'image', 0, 255, callback)
    cv2.createTrackbar('highS', 'image', 255, 255, callback)

    cv2.createTrackbar('lowV', 'image', 0, 255, callback)
    cv2.createTrackbar('highV', 'image', 255, 255, callback)
    while True:
        # grab the frame
        ret, frame = cap.read()

        # get trackbar positions
        ilowH = cv2.getTrackbarPos('lowH', 'image')
        ihighH = cv2.getTrackbarPos('highH', 'image')
        ilowS = cv2.getTrackbarPos('lowS', 'image')
        ihighS = cv2.getTrackbarPos('highS', 'image')
        ilowV = cv2.getTrackbarPos('lowV', 'image')
        ihighV = cv2.getTrackbarPos('highV', 'image')

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([ilowH, ilowS, ilowV])
        higher_hsv = np.array([ihighH, ihighS, ihighV])
        mask = cv2.inRange(hsv, lower_hsv, higher_hsv)

        frame = cv2.bitwise_and(frame, frame, mask=mask)

        # show thresholded image
        cv2.imshow('image', frame)
        k = cv2.waitKey(1) & 0xFF  # large wait time to remove freezing
        if k == 113 or k == 27:
            break
