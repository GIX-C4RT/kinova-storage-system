# import the necessary packages
from shapedetector import ShapeDetector

import imutils
import cv2
import numpy as np

vcap = cv2.VideoCapture("rtsp://192.168.1.10/color")

img_height = 892

threshold = 0
threshold_constant = 0
threshold_window = 3

while(vcap.isOpened()):
    ret, frame = vcap.read()

    image = frame
    # cv2.imshow("Raw Frame", frame)

    # crop out gripper from image
    # image is 1920 x 1080 pixels
    # print(image.shape)
    cropped_image = image[0:img_height, 0:1920]
    # cv2.imshow("Cropped Image", image)

    resized = imutils.resize(cropped_image, width=300)
    # cv2.imshow("Resized", resized)
    # cv2.createTrackbar("Height", "Resized", 20, 1080, lambda *args: None)
    # img_height = cv2.getTrackbarPos("Height", "Resized")
    ratio = cropped_image.shape[0] / float(resized.shape[0])
    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # cv2.imshow("Blurred", blurred)
    # Regular Threshold
    # thresh = cv2.threshold(blurred, threshold, 255, cv2.THRESH_BINARY)[1]
    # Otsu Threshold
    (T, thresh) = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    # Adaptive Threshold
    if threshold_window <= 1:
        threshold_window = 3
    if threshold_window % 2 != 1:
        threshold_window += 1
    
    # Adaptive Threshold Mean
    # thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, threshold_window, threshold_constant)
    # Adaptive Threshold Gaussian
    # thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, threshold_window, threshold_constant)
    thresh_window_name = "Thresholded"
    cv2.imshow(thresh_window_name, thresh)
    # cv2.createTrackbar("Threshold", thresh_window_name, 0, 255, lambda *args: None)
    # threshold = cv2.getTrackbarPos("Threshold", thresh_window_name)
    # print(threshold)
    # cv2.createTrackbar("Threshold Constant", thresh_window_name, 0, 255, lambda *args: None)
    # threshold_constant = cv2.getTrackbarPos("Threshold Constant", thresh_window_name)
    # cv2.createTrackbar("Threshold Window Size", thresh_window_name, 0, 255, lambda *args: None)
    # threshold_window = cv2.getTrackbarPos("Threshold Window Size", thresh_window_name)
    # find contours in the thresholded image and initialize the
    # shape detector
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    sd = ShapeDetector()
    # loop over the contours
    for c in cnts:
        # compute the center of the contour, then detect the name of the
        # shape using only the contour
        M = cv2.moments(c)
        try:
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
        except:
            cX = 0
            cY = 0
        shape = sd.detect(c)
        # multiply the contour (x, y)-coordinates by the resize ratio,
        # then draw the contours and the name of the shape on the image
        c = c.astype("float")
        c *= ratio
        c = c.astype("int")
        cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
        cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (255, 255, 255), 2)
        # show the output image
    # cv2.imshow("Image", image)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break