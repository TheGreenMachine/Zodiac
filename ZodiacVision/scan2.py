import cv2
import numpy as np

# cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
#
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
#
# def order_points(pts):
#     rect = np.zeros((4, 2), dtype="float32")
#     s = pts.sum(axis=1)
#     rect[0] = pts[np.argmin(s)]
#     rect[2] = pts[np.argmax(s)]
#     diff = np.diff(pts, axis=1)
#     rect[1] = pts[np.argmin(diff)]
#     rect[3] = pts[np.argmax(diff)]
#     return rect
#
# def four_point_transform(image, pts):
#     rect = order_points(pts)
#     (tl, tr, br, bl) = rect
#     widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
#     widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
#     maxWidth = max(int(widthA), int(widthB))
#     heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
#     heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
#     maxHeight = max(int(heightA), int(heightB))
#     dst = np.array([
#         [0, 0],
#         [maxWidth - 1, 0],
#         [maxWidth - 1, maxHeight - 1],
#         [0, maxHeight - 1]], dtype="float32")
#     M = cv2.getPerspectiveTransform(rect, dst)
#     warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
#     return warped
#
def hconcat_resize(img_list, interpolation=cv2.INTER_CUBIC):
    h_min = min(img.shape[0] for img in img_list)
    im_list_resize = [cv2.resize(img,
                                 (int(img.shape[1] * h_min / img.shape[0]),
                                  h_min), interpolation=interpolation)for img in img_list]
    return np.concatenate((np.asarray(im_list_resize)), axis=1)
# value = 1
# def on_change(val):
#     global value
#     value = val
# while True:
#     _, frame = cap.read()
#     imgGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     imgContours = imgGray.copy()
#     imgBlur = cv2.GaussianBlur(imgGray, (5, 5), 1)
#     imgCanny = cv2.Canny(imgBlur, 10, 50)
#     contours, _ = cv2.findContours(imgCanny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     new_contours = []
#     for c in contours:
#         peri = cv2.arcLength(c, True)
#         approx = cv2.approxPolyDP(c, 0.1 * peri, True)
#         if len(approx) == 4:
#             area = cv2.contourArea(c)
#             if area > 1000:
#                 new_contours.append(c)
#     contours = sorted(new_contours, key=cv2.contourArea, reverse=True)
#     warped = np.zeros((480, 640),  dtype=np.uint8)
#     if len(contours) > 0:
#         docCnt = cv2.approxPolyDP(contours[0], 0.1 * cv2.arcLength(contours[0], True), True)
#         warped = four_point_transform(imgGray, docCnt.reshape(4, 2))
#         print(cv2.contourArea(contours[0]))
#         cv2.drawContours(imgContours, [contours[0]], -1, (255, 0, 0), 4)
#         cv2.imwrite('warped.png', warped)
#     h_resize = hconcat_resize([imgGray, imgCanny, imgContours])
#     cv2.imshow('Frame', h_resize)
#     cv2.imshow('warp', warped)
#     cv2.waitKey(1)
frame = cv2.imread('warped.png')
imgGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
output = imgGray.copy()
retr, thresh = cv2.threshold(imgGray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
radius_list = []
for cnt in contours:
    if cv2.contourArea(cnt) > 10:
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        center = (int(x), int(y))
        radius = int(radius)
        radius_list.append(radius)
        print('Circle' + ': Center =' + str(center) + 'Radius =' + str(radius))
med = np.median(radius_list)
std = np.std(radius_list)
for cnt in contours:
    (x, y), radius = cv2.minEnclosingCircle(cnt)
    center = (int(x), int(y))
    radius = int(radius)
    if radius < med + std:
        cv2.circle(frame, center, radius, (0, 255, 0), 2)
show = hconcat_resize([imgGray,thresh, output])
cv2.imshow('Frame',show)
cv2.imshow('Contours', frame)
cv2.waitKey(0)
