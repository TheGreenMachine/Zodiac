import cv2
import yaml
import pyzed.sl as sl
from vision import *
import time

time.sleep(10)
path = 'vision.yml'
with open(path, 'r') as file:
    data = yaml.safe_load(file)
print(data)
net = networktables.NetworkTables(data, path)
net.setupCalib()
# Set configuration parameters
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
zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, data['camera']['exposure'])
runtime_parameters = sl.RuntimeParameters()
detector = detect.Detector(net)
streamer = str  eam.Streamer(data['stream']['port'])
while True:
    # time_start = time.time()
    if net.update_exposure:
        zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, net.yml_data['camera']['exposure'])
        net.update_exposure = False
    zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # A new image is available if grab() returns SUCCESS
        zed.retrieve_image(image, sl.VIEW.RIGHT)  # Retrieve the left image
        frame = image.get_data()
        mask = detector.preProcessFrame(frame)
        if mask.all() == -1:
            continue
        contour = detector.findTarget(mask, zed, point_cloud)
        stream_image = detector.postProcess(frame, contour)
        if net.calib_camera:
            streamer.write(mask)
        else:
            streamer.write(stream_image)
        # print(time.time() - time_start)
