from time import sleep

import numpy as np
import cv2


from petrone.drone import *
from petrone.protocol import *
from petrone.system import *

# global variable
#lowerThreshold = (60, 100, 15) q
#upperThreshold = (80, 255, 215)
lowerThreshold = (140, 50, 80)
upperThreshold = (180, 230, 255)
h = 0
s = 0
v = 0
r = 0
ero = 3
dil = 4
distance = 0
change = 0


def eventUpdateInformation(data):
    print("eventUpdateInformation() / {0} / {1} / {2} / Ver:{3} / 20{4:02}.{5}.{6}".format(data.modeUpdate,
                                                                                           data.deviceType,
                                                                                           data.imageType, data.version,
                                                                                           data.year, data.month,
                                                                                           data.day))


def nothing(x):
    pass


# Creating a window for later use

def createControlPanel():

    cv2.namedWindow('Control Panel')
    # Creating track bar
    # cv.CreateTrackbar(trackbarName, windowName, value, count, onChange)
    cv2.createTrackbar('hue', 'Control Panel', 0, 180, nothing)
    cv2.createTrackbar('sat', 'Control Panel', 100, 255, nothing)
    cv2.createTrackbar('val', 'Control Panel', 100, 255, nothing)
    cv2.createTrackbar('range', 'Control Panel', 50, 127, nothing)
    cv2.createTrackbar('ero', 'Control Panel', 3, 100, nothing)
    cv2.createTrackbar('dil', 'Control Panel', 3, 100, nothing)
    # cv2.createTrackbar('gauss', 'Control Panel', 5, 15, nothing)


def CalWithX(x):
    if (int(x) < 180):
        return -10
    elif (int(x) > 300):
        return 10
    else:
        return  0


def CalWithY(y):
    if (int(y) < 120):
        return 10
    elif (int(y) > 200):
        return -50
    else:
        return 0

def CalwithDistance(distance):
    if(distance < 30):
        return -10
    elif(distance > 55):
        return 10
    else:
        return 0
        #print ("hover")

if __name__ == '__main__':

    ROLL = 0
    PITCH = 0
    YAW = 0
    THROTTLE = 0
    # Drone의 객체 생성
    #drone = Drone(True, True, True, True, True)
    drone = Drone()
    # 이벤트 핸들링 함수 등록
    drone.setEventHandler(DataType.UpdateInformation, eventUpdateInformation)

    # 장치에 연결
    drone.connect()  # 마지막으로 연결된 시리얼 포트와 검색된 장치(페트론) 중 가장 신호가 강한 장치에 연결
    # drone.connect(portName="COM14", deviceName="PETRONE 6504")      # 시리얼 포트와 장치(페트론)를 지정하여 연결
    sleep(1)
    # 장치에 연결된 경우 정보 요청
    if drone.isConnected():
        drone.sendControl(0, 0, 0, 0)
        sleep(1)
        #createControlPanel()
        print("start flying")
        drone.sendTakeOff()
        sleep(4)
        cap = cv2.VideoCapture('rtsp://192.168.100.1/cam1/mpeg4')
        cnt = 0
        while (cap.isOpened()):
            '''
            h = cv2.getTrackbarPos('hue', 'Control Panel')
            s = cv2.getTrackbarPos('sat', 'Control Panel')
            v = cv2.getTrackbarPos('val', 'Control Panel')
            r = cv2.getTrackbarPos('range', 'Control Panel')
            ero = cv2.getTrackbarPos('ero', 'Control Panel')
            dil = cv2.getTrackbarPos('dil', 'Control Panel')
            # gaussValue = cv2.getTrackbarPos('gauss', 'Control Panel')

            lowerThreshold = (h - 10, s - r, v - r)
            upperThreshold = (h + 10, s + r, v + r)
            '''

            cnt += 1
            e1 = cv2.getTickCount()
            ret, frame = cap.read()
            origin = frame
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lowerThreshold, upperThreshold)

            # be comented part
            '''

            #blur Image
            #frame = cv2.blur(origin,(5,5))

            if you want to add more color in same time set another threshold and make mask and merge it
            mask2= cv2.inRange(hsv,lowerThreshold2,upperThreshold2)
            cv2.addWeighted(mask,1.0,mask2,1.0,0.0,mask)

            '''
            # erode =  cut masked image' boundary
            mask = cv2.erode(mask, None, iterations=ero)
            # dilate = expend masked image
            mask = cv2.dilate(mask, None, iterations=dil)
            frame = cv2.bitwise_and(frame, frame, mask=mask)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None
            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                if int(M["m00"]) > 0:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if radius > 10:
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    distance = 2168*radius**(-1.09)
                    print ("distance", distance)
                    # cv2.circle(frame, center, 5, (0, 0, 255), -1)
                    # pts.appendleft(center)

                    if (cnt > 2):
                        cnt = 0
                        print('move')
                        ROLL = CalWithX(x)
                        THROTTLE = CalWithY(y)
                        PITCH = CalwithDistance(distance)
                        print("roll : ", ROLL,", pitch : ", PITCH,", yaw : ", YAW,", throttle : ", THROTTLE)
                        drone.sendControl(ROLL, PITCH, YAW, THROTTLE)


            e2 = cv2.getTickCount()
            time = (e2 - e1) / cv2.getTickFrequency()
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, format(time * 1000), (0, 60), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, format(distance), (0, 100), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            #cv2.putText(frame, format(center), (0, 100), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            #cv2.putText(frame, format("roll : ", ROLL,", pitch : ", PITCH,", yaw : ", YAW,", throttle : ", THROTTLE), (0, 20), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)



            cv2.imshow('frame', frame)
            cv2.imshow('orign', origin)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            if cv2.waitKey(1) & 0xFF == ord('Q'):
                break

        cap.release()
        cv2.destroyAllWindows()

        # 장치 연결 해제
        print("Disconnect device.")
        drone.sendLinkDisconnect()
        sleep(0.2)

    drone.close()
