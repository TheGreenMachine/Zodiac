from vision import camera
import cv2
cam = camera.Camera()

# cap = cv2.VideoCapture(0)
# _, frame = cap.read()

frame = cam.read()
cv2.imshow('Frame', frame)
cv2.waitKey(0)