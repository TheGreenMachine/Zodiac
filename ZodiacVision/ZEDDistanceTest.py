import cv2
import pyzed.sl as sl
import numpy as np
import math
import sys

zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.RESOLUTION_VGA
init_params.depth_mode = sl.DEPTH_MODE.DEPTH_MODE_ULTRA  # Use PERFORMANCE depth mode
init_params.coordinate_units = sl.UNIT.UNIT_INCH  # Use milliliter units (for depth measurements)

# Open the camera
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    exit(1)
# Create and set RuntimeParameters after opening the camera
runtime_parameters = sl.RuntimeParameters()
point_cloud = sl.Mat()
image = sl.Mat()
while True:
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.VIEW_RIGHT)
        zed.retrieve_measure(point_cloud, sl.MEASURE.MEASURE_XYZRGBA)
        x = round(image.get_width() / 2)
        y = round(image.get_height() / 2)
        err, point3D = point_cloud.get_value(x, y)

        distance = math.sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1] + point3D[2]*point3D[2])


        print("Distance to Camera at ({0}, {1}): {2} M".format(x, y, distance))
        sys.stdout.flush()
