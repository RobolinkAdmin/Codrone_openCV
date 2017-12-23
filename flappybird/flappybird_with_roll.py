"""
detecting wing
"""

import cv2
import keyControl as kc
from CoDrone.codrone import *

"""
DIFFER_PERCENT = 0.9
MOVING_THRESH = 10000
ROLL_THRESH = 10
THRO_THRESH = 20
ROLL_MOTOR = 5
THRO_MOTOR = -5
"""

def nothing(none):
    pass

def controlPanel(differ_limit,moving_limit,roll_limit,throttle_limit ,roll_motor,throttle_motor):
    # Creating a window for later use
    cv2.namedWindow('Control Panel')

    # Creating track bar
    cv2.createTrackbar('differ_percent', 'Control Panel', int(differ_limit*10), 10, nothing)
    cv2.createTrackbar('moving_threshold', 'Control Panel', moving_limit, 20000, nothing)
    cv2.createTrackbar('roll_threshold', 'Control Panel', roll_limit, 1000, nothing)
    cv2.createTrackbar('throttle_threshold', 'Control Panel', throttle_limit, 1000, nothing)
    cv2.createTrackbar('roll_motor', 'Control Panel', roll_motor, 10, nothing)
    cv2.createTrackbar('throttle_motor', 'Control Panel', throttle_motor, 10, nothing)


def getThresholds():
    # get info from track bar and appy to result
    #store each trackbar position into variables
    differ_percent = cv2.getTrackbarPos('differ_percent', 'Control Panel')
    moving_threshold = cv2.getTrackbarPos('moving_threshold', 'Control Panel')
    roll_threshold = cv2.getTrackbarPos('roll_threshold', 'Control Panel')
    thro_threshold = cv2.getTrackbarPos('throttle_threshold', 'Control Panel')
    roll_motor = cv2.getTrackbarPos('roll_motor', 'Control Panel')
    thro_motor = cv2.getTrackbarPos('throttle_motor', 'Control Panel')
    return differ_percent * 0.1, moving_threshold,roll_threshold,thro_threshold,roll_motor,thro_motor

def resize_frame(frame, dividing_n):
    height, width, layers = frame.shape
    width = int(width / dividing_n)
    height = int(height / dividing_n)
    frame = cv2.resize(frame, (width, height))
    return frame, height, width

#for debug
def test_drone_moving(drone, time, count, roll, pitch, yaw, throttle):
    freq = time * cv2.getTickFrequency()
    for i in range(count):
        e1 = cv2.getTickCount()
        e2 = 0
        while (e2 - e1 < freq):
            e2 = cv2.getTickCount()
        drone.sendControl(roll, pitch, yaw, throttle)

# drone mode : 1
def processing_capture(mode = 0, test = 0):
    frame_cnt = 0

    drone = CoDrone()
    if( mode == 1):
        drone.connect()
        sleep(3)
        if drone.isConnected():
            drone.sendTakeOff()
            sleep(5)

    cap = cv2.VideoCapture(0)

    dividingFactor = 2
    pre_gray = 0

    if(cap.isOpened()):
        ret, frame = cap.read()
        frame,height,width = resize_frame(frame, dividingFactor)
        pre_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    ##########################
    #for testing
    while(test == 1):
        if(mode == 1 and not drone.isConnected()):
            drone.close()
            print("die!")
            return

        ret, frame = cap.read()
        cv2.imshow('frame', frame)
        # time, count, roll, pitch, yaw, throttle
        if(kc.keyBoardController(drone,cap,cv2.waitKey(1) & 0xFF) ):
            print("what?")
            break
    #
    ###################################

    left_cnt = 0
    right_cnt = 0
    up_cnt = 0
    left_area = 0
    right_area = 0

    throttle = 0
    roll = 0
    pitch = 0
    yaw = 0
    LEFT_ADD = -5
    DOWN_ADD = -20

    #to fit parameter to drone
    controlPanel(differ_limit=0.8, moving_limit=8000, roll_limit=7, throttle_limit=20,roll_motor=20,throttle_motor=20)
    while (cap.isOpened()):  # while capturing video repeat this loop

        #start time
        e1 = cv2.getTickCount()
        frame_cnt += 1

        #resizing frame
        ret, frame = cap.read()
        frame,height,width = resize_frame(frame, dividingFactor)

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # detecting moving
        frame = cv2.absdiff(pre_gray, gray)

        # create black and white threshold 40/255. anything with
        # grascale value above 40 will be pushed to pure white 255
        binary_frame = cv2.threshold(frame, 40, 255, cv2.THRESH_BINARY)[1]

        #divide frame with left and right
        right_frame = binary_frame[:, int(width / 2 + width * 0.1):]
        left_frame = binary_frame[:,0:int(width/2 - width*0.1)]
        left_area += cv2.countNonZero(left_frame)
        right_area += cv2.countNonZero(right_frame)

        DIFFER_PERCENT, MOVING_THRESH,ROLL_THRESH,THRO_THRESH,ROLL_MOTOR,THRO_MOTOR = getThresholds()

        if frame_cnt == 5:
            # + : right
            # - : left
            area_differ = left_area - right_area
            area_sum = left_area + right_area
            DIFFER_THRESH = DIFFER_PERCENT * area_sum

            ###ROLL
            #right
            if (area_differ > DIFFER_THRESH):
                left_cnt = 0
                if(right_cnt < ROLL_THRESH):
                    roll = ROLL_MOTOR
                    right_cnt += 1
                    throttle = -5
                else:
                    roll = 0
            #left
            elif (area_differ < -DIFFER_THRESH):
                right_cnt = 0
                if(left_cnt < ROLL_THRESH):
                    roll = -ROLL_MOTOR + LEFT_ADD
                    left_cnt += 1
                    throttle = -5
                else:
                    roll = 0
            #middle
            else:
                roll = 0
                if(area_sum > MOVING_THRESH):
                    if(up_cnt < THRO_THRESH):
                        throttle = THRO_MOTOR
                        up_cnt += 1
                    else:
                        throttle = 0
                else:
                    throttle = -THRO_MOTOR + DOWN_ADD
                    up_cnt = 0

            # time
            e2 = cv2.getTickCount()
            time = (e2 - e1) / cv2.getTickFrequency()
            fps = 1 / time

            print("===================")
            if roll > 0: print("<<<===================")
            if roll < 0: print("===================>>>")
            print("area_differ", area_differ)
            print("area_sum ",area_sum)
            print("up_cnt",up_cnt)
            print("right_cnt ", right_cnt)
            print("left_cnt ", left_cnt)
            print("time", time)
            print(roll,pitch, yaw,throttle)
            print("===================")

            if mode == 1: drone.sendControl(roll, pitch, yaw, throttle)

            frame_cnt = 0
            left_area = 0
            right_area = 0

            # put text on frame you pass in
            kc.putTextonFrame(binary_frame, fps, roll, pitch, yaw, throttle)
            cv2.imshow('frame', binary_frame)
            if (kc.keyBoardController(drone, cap, cv2.waitKey(1) & 0xFF)):
                break
        #for next comparision
        pre_gray = gray

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    processing_capture(1,0)