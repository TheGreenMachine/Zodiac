import cv2
import pyzed.sl as sl


# Team 1816's 2020 Vision Processing Code
# Used with OpenCV and Gstreamer

def preProcess(image):
    lower_color = (60, 106, 150)
    upper_color = (130, 255, 255)

    h, w, _ = image.shape
    image = image[0:int(h/2), 0:w]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    return mask


def findTarget(image):
    return (x, y)


def postProcess(image):
    return image


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)

    while 1:
        _, frame = cap.read()
        preProcessImage = preProcess(frame)
        cv2.imshow("PreProcessed Image", preProcessImage)
        cv2.waitKey(1)
        postProcessImage = postProcess(preProcessImage)
