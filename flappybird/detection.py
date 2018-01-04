import cv2
import numpy as np
import math

def segment(image):
    threshold = cv2.threshold(image, 100, 255, cv2.THRESH_BINARY)[1]
    _, cnts, _ = cv2.findContours(threshold,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(cnts) == 0:
        return
    else:
        segmented = max(cnts, key=cv2.contourArea)
    cv2.imshow("i", threshold)
    cv2.waitKey(1)
    return segmented

def count(segmented):
    chull = cv2.convexHull(segmented)


face_detector = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
def face(image, window,ROI):
    H,W,_=image.shape
    H_half = int(H/2)
    W_half = int(W/2)
    roi = image[0:H_half, W_half - ROI:W_half + ROI]
    faces = face_detector.detectMultiScale(roi, 1.3, 5, 10)

    if(len(faces) == 0): return False
    print(faces)
    for (x, y, w, h) in faces:
        # Crop the image frame into rectangle
        for i in range(0,10,2):
            cv2.rectangle(image, (W_half - ROI + x+i, y+i), (W_half - ROI + x + w-i, y+h-i), (255 - 10*i, 255 - 10*i, 255 - 10*i), 1)

            #cv2.rectangle(image, (W_half - ROI, 0), (W_half + ROI, H_half), (255-i, 255-i, 255-i), 1)

        image = cv2.flip(image,1)
        cv2.imshow(window, image)
        cv2.waitKey(1)
        return True
