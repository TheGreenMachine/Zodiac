import cv2

# Open the stream via GStreamer. Note that the pipeline ends with an appsink
# element, which OpenCV reads from. The sync and drop options here instruct
# Gstreamer to not block the program waiting for new frames and to drop
# frames if OpenCV cannot read them quickly enough.
def makecap():
    return cv2.VideoCapture(
        'rtspsrc location=rtsp://127.0.0.1:5800/test latency=0 transport=tcp ! rtph264depay ! decodebin ! tee name=t ! queue ! videoconvert ! appsink sync=false drop=true t. ! queue ! videoconvert ! autovideosink',
        cv2.CAP_GSTREAMER
    )

cap = makecap()
while True:
    successful, frame = cap.read()
    if successful: # Display the frame
        cv2.imshow('Frame', frame)
        if cv2.waitKey(10) == ord('q'):
            break # Exit the program if the key q is pressed
    else: # If a frame can't be read try restarting the stream
        cap = makecap()
