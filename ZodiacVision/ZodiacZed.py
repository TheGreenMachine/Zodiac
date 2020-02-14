import cv2
import math
import numpy as np
import networktables as nt
import argparse
import pyzed.sl as sl
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
        if match < 10:
            cx = rect[0] + (rect[2] * .5)
            cy = rect[1] + (rect[3] * .5)
            nt_table.putNumber('center_x', cx)
            nt_table.putNumber('center_y', cy)
            err, point3D = point_cloud.get_value(cx, cy)
            distance = math.sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1] + point3D[2] * point3D[2])
            if not math.isnan(distance):
                nt_table.putNumber('distance', round(distance))
            return largest
    clearNetworkTables(nt_table)
    return -1


def postProcess(image, target):
    if target is -1:
        return image
    drawnimage = cv2.drawContours(image, [target], -1, (0, 255, 255), 2)
    return drawnimage


def clearNetworkTables(table):
    table.putNumber('center_x', -1)
    table.putNumber('center_y', -1)
    table.putNumber('distance', -1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--view", help="enable windows")
    args = parser.parse_args()

    fps = 100
    frame_width = 672
    frame_height = 376
    zed = sl.Camera()
    point_cloud = sl.Mat()
    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.INCH  # Use milliliter units (for depth measurements)
    init_params.camera_resolution = sl.RESOLUTION.VGA
    init_params.camera_fps = 100

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(-1)
    image = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    gst_str_rtp = "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=10.18.16.5 port=1180"
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
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # A new image is available if grab() returns SUCCESS
            zed.retrieve_image(image, sl.VIEW.LEFT)  # Retrieve the left image
            frame = image.get_data()
            preProcessImage = preProcess(frame)
            if args.view:
                cv2.imshow("PreProcessed Image", preProcessImage)
            target = findTarget(preProcessImage, hexes, table)
            if args.view:
                postProcessImage = postProcess(frame, target)
                cv2.imshow("PostProcessed Image", postProcessImage)
                cv2.waitKey(1)
            # Write to SHM
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            out.write(frame)
