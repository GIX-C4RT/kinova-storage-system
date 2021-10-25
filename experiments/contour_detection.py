import cv2
import numpy as np
# cap = cv2.VideoCapture("rtsp://192.168.1.10/color")
# while(cap.isOpened()):
#     ret, frame = cap.read()
#     cv2.imshow('frame', frame)
#     if cv2.waitKey(20) & 0xFF == ord('q'):
#         break
# cap.release()
# cv2.destroyAllWindows()

vcap = cv2.VideoCapture("rtsp://192.168.1.10/color")

while(vcap.isOpened()):
    ret, frame = vcap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(gray,50,255,cv2.THRESH_BINARY)
    contours,hierarchy = cv2.findContours(thresh, 1, 2)
    # cnt = contours[0]
    # rect = cv2.minAreaRect(cnt)
    # box = cv2.boxPoints(rect)
    # box = np.int0(box)
    # cv2.drawContours(frame,[box],-1,(0,0,255),2)
    cv2.drawContours(frame,contours,-1,(0,0,255),2)
    cv2.imshow('frame', frame)
    # cv2.waitKey(0)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()