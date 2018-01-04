"""
detecting wing
"""

import cv2
from keyControl import *
from CoDrone.codrone import *


CurrentBattV = 0


"""
DIFFER_PERCENT = 0.9
MOVING_THRESH = 10000
ROLL_THRESH = 10
THRO_THRESH = 20
ROLL_MOTOR = 5
THRO_MOTOR = -5
"""
# method that takes in the amount of time that passes per loop and adds it
# given an amount of time we want to reset the timer to 0.
def timeCounter(elapsedTime, timeSum, resetTime):
    # elapsed time will be the amount of time the while loop takes
    timeSum = elapsedTime + timeSum

    # check to see if the time sum is greater than the reset time
    if (timeSum > resetTime):
        # in our case we want the messages to be sent every 50 milliseconds
        timeSum = 0
        # once we reach the resettime we set it back to zero

    return timeSum
    # make sure to retunr the amount of time elapsed


def requestAndPrint(drone, data, eventUpdate):
    drone.sendRequest(data)
    drone.setEventHandler(data, eventUpdate)

def eventUpdateBattery(data):
    global CurrentBattV
    '''
            self.adjustGradient             = 0
            self.adjustYIntercept           = 0
            self.gradient                   = 0
            self.yIntercept                 = 0
            self.flagBatteryCalibration     = False
            self.batteryRaw                 = 0
            self.batteryPercent             = 0
            self.voltage                    = 0
Variable name	form	range	size	Explanation
adjustGradient	Int16	-	2 Byte	Tilt adjustment value
adjustYIntercept	Int16	-	2 Byte	Y intercept adjustment value
gradient	Int16	-	2 Byte	inclination
yIntercept	Int16	-	2 Byte	Y intercept
flagBatteryCalibration	Bool	True / False	1 Byte	Battery Calibration Complete
batteryRaw	Int32	0 ~ 4096	4 Byte	Battery ADC Raw Value
batteryPercent	Int8	0 ~ 100	1 Byte	Battery% Value
voltage	Int16	0 ~ 4.5	2 Byte	Battery output value converted to voltage
    '''

    print("grad {0} yint {1} Agrad {2} Ayint {3} flag {4} battRaw {5} battPercent {6} MilliVolts {7}0   ".format(
        data.adjustGradient, data.adjustYIntercept, data.gradient
        , data.yIntercept, data.flagBatteryCalibration, data.batteryRaw
        , data.batteryPercent, data.voltage))
    CurrentBattV = data.batteryPercent



def nothing(none):
    pass

def controlPanel(differ_limit,moving_limit,roll_limit,throttle_limit ,roll_motor,throttle_motor):
    # Creating a window for later use
    cv2.namedWindow('Control Panel')

    # Creating track bar
    cv2.createTrackbar('differ_percent', 'Control Panel', int(differ_limit*10), 10, nothing)
    cv2.createTrackbar('moving_threshold', 'Control Panel', moving_limit, 20000, nothing)
    cv2.createTrackbar('roll_threshold', 'Control Panel', int(roll_limit*10), 1000, nothing)
    cv2.createTrackbar('throttle_threshold', 'Control Panel', int(throttle_limit*10), 1000, nothing)
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
    return differ_percent * 0.1, moving_threshold,roll_threshold * 0.1 ,thro_threshold *0.1,roll_motor,thro_motor

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

#NEED TO DO
#do i have to connect every time?
def make_drone(port_name,drone_name):
    drone = CoDrone(1,1,1,1,1)
    drone.connect(port_name, drone_name)
    sleep(3)
    while not drone.isConnected():
        print("Drone is not connected!")
        drone.close()
        drone.connect(port_name, drone_name)
        sleep(3)

    print("Connect!")
    return drone


# drone mode : 1
def processing_capture(mode = 0, test = 0):
    frame_cnt = 0

    if(mode == 1):
        drone = make_drone("COM8","PETRONE 8097")
        drone.sendTakeOff()
        sleep(5)

    cap = cv2.VideoCapture(0)
    
    dividingFactor = 2
    pre_gray = 0

    if(cap.isOpened()):
        ret, frame = cap.read()
        height_do, width_do,_ = frame.shape
        frame,height,width = resize_frame(frame, dividingFactor)
        pre_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    left_cnt = 0
    right_cnt = 0
    up_cnt = 0
    left_area = 0
    right_area = 0
    throttle = 0
    roll = 0
    pitch = 0
    yaw = 0
    DOWN_ADD = -10
    batteryRequestTimer = 0
    timePassed = 0

    e1 = cv2.getTickCount()
    #to fit parameter to drone
    controlPanel(differ_limit=0.8, moving_limit=5000, roll_limit=0.5, throttle_limit=1.0,roll_motor=30,throttle_motor=80)
    while (cap.isOpened()):  # while capturing video repeat this loop

        #start time
        frame_cnt += 1

        #resizing frame
        ret, frame = cap.read()

        e1 = cv2.getTickCount()
        frame_,height,width = resize_frame(frame, dividingFactor)

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame_, cv2.COLOR_BGR2GRAY)

        # detecting moving
        gray_ = cv2.absdiff(pre_gray, gray)
        e2 = cv2.getTickCount()


        # create black and white threshold 40/255. anything with
        # grascale value above 40 will be pushed to pure white 255
        binary_frame = cv2.threshold(gray_, 40, 255, cv2.THRESH_BINARY)[1]

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
            if(area_sum > MOVING_THRESH):
                if (area_differ < -DIFFER_THRESH):
                    left_cnt = 0
                    if(right_cnt < ROLL_THRESH):
                        roll = ROLL_MOTOR
                        right_cnt += timePassed
                        throttle = 10
                        cv2.rectangle(frame, (0, 0), (10, height_do), (10, 200, 10), 3)
                    else:
                        roll = 0
                #left
                elif (area_differ > DIFFER_THRESH):
                    right_cnt = 0
                    if(left_cnt < ROLL_THRESH):
                        roll = -ROLL_MOTOR
                        left_cnt += timePassed
                        throttle = 10
                        cv2.rectangle(frame, ((width_do)-10, 0), (width_do, height_do), (10, 200, 10), 3)
                    else:
                        roll = 0
                #up
                else:
                    roll = 0
                    if (up_cnt < THRO_THRESH):
                        up_cnt += timePassed
                        cv2.rectangle(frame, (0, 0), (width_do, 10), (10, 200, 10), 3)
                        throttle = THRO_MOTOR
                        pitch = 20
            #down
            else:
                cv2.rectangle(frame, (width-10, height-10), (width, height), (10, 200, 10), 3)
                throttle = -THRO_MOTOR + DOWN_ADD
                up_cnt = 0
            """
            print("move_sum", area_sum)
            print("move_differ", area_differ)
            print("up_cnt", up_cnt)
            print("left_cnt", left_cnt)
            print("right_cnt", right_cnt)
            """
            #timer
            if batteryRequestTimer == 0 and mode == 1:
                requestAndPrint(drone,DataType.Battery, eventUpdateBattery)
                sleep(0.03)

            # time stuff-----------------------------------------------------------------------
            e2 = cv2.getTickCount()
            timePassed = (e2 - e1) / cv2.getTickFrequency()
            e1 = e2
            fps = 1 / timePassed
            # Set the update times for each of the following the bigger the time
            # the less frequent the data will updated
            batteryRequestTimer = timeCounter(timePassed, batteryRequestTimer, 1)  # every 700 milliseconds
            #------------------------------------------------------------------------------------


            if mode == 1: drone.sendControl(roll, pitch, yaw, throttle)
            pitch = 0
            frame_cnt = 0
            left_area = 0
            right_area = 0

            frame = cv2.flip(frame,1)
            # put text on frame you pass in
            putTextonFrame(frame= frame, battPercent_ = CurrentBattV * 10, differ_ =area_differ, sum_=area_sum, THRE_DIFFER=DIFFER_THRESH, THRE_SUM=MOVING_THRESH)

            cv2.imshow('frame', frame)
            if (mode ==1 and keyBoardController(drone, cv2.waitKey(1) & 0xFF)):
                break
            elif(mode == 0):
                cv2.waitKey(1)
        #for next comparision
        pre_gray = gray

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    #parameter : is drone version? , is test?
    processing_capture(1,0)