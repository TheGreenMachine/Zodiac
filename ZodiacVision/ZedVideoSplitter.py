import cv2
import argparse

# Zed Video Splitter
# Designed to take in an .avi LEFT-RIGHT video and output it into separate videos

parser = argparse.ArgumentParser()
parser.add_argument("--source", default=0,
                    help="Input Source")
args = parser.parse_args()

cap = cv2.VideoCapture(args.source)
_, frame = cap.read()
(h, w) = frame.shape[:2]
left_frame_writer = cv2.VideoWriter('Left_Frame.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 100, (int(w / 2), h))
right_frame_writer = cv2.VideoWriter('Right_Frame.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 100, (int(w / 2), h))
while True:
    retr, frame = cap.read()
    if retr == True:
        left_frame = frame[0:h, 0:int(w / 2)]
        left_frame_writer.write(left_frame)
        right_frame = frame[0:h, int(w / 2):w]
        right_frame_writer.write(right_frame)
        cv2.imshow("Left Frame", left_frame)
        cv2.imshow("Right Frame", right_frame)
        cv2.waitKey(1)
    else:
        break
left_frame_writer.release()
right_frame_writer.release()
cap.release()
