import cv2
import numpy as np

def display(im, bbox):
    n = len(bbox)
    for j in range(n):
        cv2.line(im, tuple(bbox[j][0]), tuple(bbox[ (j+1) % n][0]), (255,0,0), 3)

    # Display results
    cv2.imshow("Results", im)

qrDecoder = cv2.QRCodeDetector()

cap = cv2.VideoCapture("rtsp://192.168.1.10/color")

while(cap.isOpened()):
    ret, frame = cap.read()
    # Detect and decode the qrcode
    data,bbox,rectifiedImage = qrDecoder.detectAndDecode(frame)
    if len(data)>0:
        print("Decoded Data : {}".format(data))
        display(frame, bbox)
        rectifiedImage = np.uint8(rectifiedImage);
        cv2.imshow("Rectified QRCode", rectifiedImage);
    else:
        print("QR Code not detected")
        cv2.imshow("Results", frame)


    # cv2.imshow('frame', frame)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()