import cv2
from petrone.drone import *
from petrone.protocol import *


#drone = Drone(True,True,True,True,True)
#drone = Drone()
#sleep(1)
#drone.connect()
#sleep(2)

if drone.isConnected():
   cap = cv2.VideoCapture(0)
   drone.sendControl(0,0,0,0)
   sleep(1)
   print(“start flying”)
   drone.sendTakeOff()
   mode = 1
   # Create a while loop to grab frame by frame of video
   while (cap.isOpened())  :  # while capturing video repeat this loop
       e1 = cv2.getTickCount()
       if mode == 1:
           framecnt += 1
       ret1, frame1 = cap.read()
       ret2, frame2 = cap.read()

       height, width, layers = frame1.shape

       # comment this line if you want the fullsize window
       # we divide to reduce data coming in
       frame1s = cv2.resize(frame1, (int(width / dividingFactor), int(height / dividingFactor)))
       frame2s = cv2.resize(frame2, (int(width / dividingFactor), int(height / dividingFactor)))


       # Convert frame to grayscale
       gray1 = cv2.cvtColor(frame1s, cv2.COLOR_BGR2GRAY)
       gray2 = cv2.cvtColor(frame2s, cv2.COLOR_BGR2GRAY)

       frame = cv2.absdiff(gray1 ,gray2)

       # create black and white threshold 40/255. anything with
       # grascale value above 40 will be pushed to pure white 255

       binary_frame = cv2.threshold(frame, 40, 255, cv2.THRESH_BINARY)[1]

       cnts = cv2.findContours(binary_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
       #print (len(cnts))
       framewind += len(cnts)
       if mode ==1 and framecnt == 2:
           if framewind >400:
               drone.sendControl(0,30,0,40)
               print(‘up’)
           else:
               drone.sendControl(0,0,0,-50)
               print(‘down’)
           framewind = 0
           framecnt = 0
       # display the original frame
       cv2.imshow(‘Frame’, binary_frame)

       # In order to quit the while loop click q
       # to break out of the loo
       if cv2.waitKey(1) & 0xFF == ord(‘q’):
           break
       if cv2.waitKey(1) & 0xFF == ord(‘s’):
           mode = 1
           print(‘active mode’)
       if cv2.waitKey(1) & 0xFF == ord(‘d’):
           mode = 0
           drone.sendControl(0,0,0,0)
           print(‘deactive mode’)


       e2 = cv2.getTickCount()
       time = (e2 - e1) / cv2.getTickFrequency()

       # print(‘{:02.2f}’.format(1/(time)))#in milliseconds

       # Relase the camera
   drone.sendLanding()
   sleep(1)
   cap.release()
   sleep(1)
   drone.sendLinkDisconnect()
   sleep(1)
drone.close()