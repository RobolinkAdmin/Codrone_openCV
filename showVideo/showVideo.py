import numpy as np
import cv2

#Capture video from the Wifi Connection to FPV module
cap = cv2.VideoCapture('rtsp://192.168.100.1/cam1/mpeg4')


#Create a while loop to grab frame by frame of video
while (cap.isOpened()):#while capturing video repeat this loop

    #save the video frame read into variable "frame"
    ret, frame = cap.read()

    #show the video variable "frame" on a windows with
    #the title video frame
    #cv2.imshow('Windows Name', frameThatyouWanttoDisplay)
    cv2.imshow('video Frame', frame)


    #In order to quit the while loop click q or Q
    #to break out of the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if cv2.waitKey(1) & 0xFF == ord('Q'):
        break

#Relase the camera
cap.release()

#close all the windows generated
cv2.destroyAllWindows()