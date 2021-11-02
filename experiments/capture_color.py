import cv2
import numpy as np
cap = cv2.VideoCapture("rtsp://192.168.1.10/color")
while(cap.isOpened()):
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()