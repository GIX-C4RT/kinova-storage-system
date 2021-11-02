import cv2
import numpy as np
cap = cv2.VideoCapture("rtsp://192.168.1.10/depth")
while(cap.isOpened()):
    # capture frame
    ret, frame = cap.read()
    # print(type(frame))
    # print(frame.dtype)
    cv2.imshow('frame', frame)
    # print(frame.max())
    # convert frame type from int to float
    float_frame = frame.astype(float)
    cv2.imshow("float frame", float_frame)
    # print(float_frame.max())
    # normalize array values to between 0 and 1
    norm_frame = float_frame / float_frame.max()
    cv2.imshow("norm frame", norm_frame)
    # print(norm_frame.max())
    # min, max = np.min(frame), np.max(frame)
    # print(min,max)
    # full_range = (float_frame - min) / (max - min) * 255
    # cv2.imshow("full range", full_range)
    # scaled_range = frame * 50
    # cv2.imshow("scaled range", scaled_range)



    if cv2.waitKey(20) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()