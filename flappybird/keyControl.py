from time import sleep
import cv2

point_color = (100,255,100)
font_big = 1.5
simple_color = (0,0,10)
font_small = 0.5

def nothing(x):
    pass

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


def hsvControlPanel(initialHue, initialSaturation, initialValue, initialSatRange, initialValRange, initialHueRange):
    # Creating a window for later use
    cv2.namedWindow('Control Panel')

    # Creating track bar
    # cv.CreateTrackbar(trackbarName, windowName, value, count, onChange)  None
    cv2.createTrackbar('Hue', 'Control Panel', initialHue, 180, nothing)  # default 0 205 255 69 8 12
    cv2.createTrackbar('Saturation', 'Control Panel', initialSaturation, 255, nothing)
    cv2.createTrackbar('Value', 'Control Panel', initialValue, 255, nothing)
    cv2.createTrackbar('Sat Range', 'Control Panel', initialSatRange, 127, nothing)
    cv2.createTrackbar('Val Range', 'Control Panel', initialValRange, 127, nothing)
    cv2.createTrackbar('Hue Range', 'Control Panel', initialHueRange, 50, nothing)


def getThresholds():
    # get info from track bar and appy to result
    # store each trackbar position into variables
    h = cv2.getTrackbarPos('Hue', 'Control Panel')
    s = cv2.getTrackbarPos('Saturation', 'Control Panel')
    v = cv2.getTrackbarPos('Value', 'Control Panel')
    sr = cv2.getTrackbarPos('Sat Range', 'Control Panel')
    vr = cv2.getTrackbarPos('Val Range', 'Control Panel')
    hr = cv2.getTrackbarPos('Hue Range', 'Control Panel')

    # the hue has a range of +-10
    # the saturation has range +- sat range same for value
    lowerThreshold = (h - hr, s - sr, v - vr)
    upperThreshold = (h + hr, s + sr, v + vr)
    # We calculate the thresholds depending on the user input

    return lowerThreshold, upperThreshold

def putTextonFrame(frame, battPercent_ = 0, differ_ = 0, sum_ = 0, THRE_DIFFER = 0, THRE_SUM = 0):
    font = cv2.FONT_HERSHEY_SIMPLEX
    h,w,_ = frame.shape
    w = int(w/15)

    if(battPercent_ < 10):
        color_ = point_color
        font_size = font_big
    else:
        color_ = simple_color
        font_size = font_small

    cv2.putText(frame, "Batt : " +  format(battPercent_), (0, 1*w), cv2.FONT_HERSHEY_SIMPLEX, font_size , color_, 1.5, cv2.LINE_AA)

    differ_ = abs(differ_)
    if(THRE_SUM < sum_ and THRE_DIFFER < differ_):
        color_ = point_color
        font_size = font_big
    else:
        color_ = simple_color
        font_size = font_small

    if(THRE_DIFFER < 1): THRE_DIFFER = 1
    cv2.putText(frame, "Differ : "  + "{0:.2f}".format((differ_/THRE_DIFFER) * 100,2) + "%", (0, 2*w), font, font_size, color_, 1.5, cv2.LINE_AA)

    if(THRE_SUM < sum_):
        color_ = point_color
        font_size = font_big
    else:
        color_ = simple_color
        font_size = font_small

    cv2.putText(frame, "Sum : " + "{0:.2f}".format(sum_/THRE_SUM * 100) + "%", (0, 3*w), cv2.FONT_HERSHEY_SIMPLEX, font_size, color_, 1.5, cv2.LINE_AA)



# this method takes in the the key pressed and
# returns 0 or 1 iuf you want to break out of the loop
def keyBoardController(drone, key):
    breakLoop = 0
    global throttle
    global yaw
    global pitch
    global roll

    # THROTTLE
    # ----------------------------------------------------------
    if key == ord("w"):
        # drone.sendControl(roll_, pitch_, yaw_, throttle_)
        drone.sendControl(0, 0, 0, 50)
        throttle = 50
    # ----------------------------------------------------------
    if key == ord("s"):
        drone.sendControl(0, 0, 0, -50)
        throttle = -50
    ##YAW
    # ----------------------------------------------------------
    if key == ord("a"):
        drone.sendControl(0, 0, 50, 0)
        yaw = 50
    # ----------------------------------------------------------
    if key == ord("d"):
        drone.sendControl(0, 0, -50, 0)
        yaw = -50
    ##PITCH
    # ----------------------------------------------------------
    if key == ord("i"):
        drone.sendControl(0, 50, 0, 0)
        pitch = 50
    # ----------------------------------------------------------
    if key == ord("k"):
        drone.sendControl(0, -50, 0, 0)
        pitch = -50

    ##ROLL
    # ----------------------------------------------------------
    if key == ord("j"):
        drone.sendControl(-50, 0, 0, 0)
        roll = -50
    # ----------------------------------------------------------
    if key == ord("l"):
        drone.sendControl(50, 0, 0, 0)
        roll = 50

    # ----------------------------------------------------------

    ##CONNECTING
    # ----------------------------------------------------------
    if key == ord("r"):
        print("CONNECTING TO CODRONE")
        drone.connect()
        sleep(0.5)
        drone.sendControl(0, 0, 0, 0)  # (roll, pitch, yaw, throttle)
        sleep(1)
        print("CONNECT------")


    if key == ord("t"):
        print("TAKEOFF CODRONE")
        drone.sendTakeOff()
        sleep(5)
        print("TAKEOFF -----")


    if key == ord("y"):
        drone.sendLanding()
        sleep(1)
        print("[LANDING CODRONE]")
        drone.sendControl(0, 0, 0, 0)

    if key == ord("u"):
        drone.sendStop()
        sleep(1)
        print("[STOPPING CODRONE]")
        drone.sendControl(0, 0, 0, 0)

    if key == ord("q"):
        drone.sendStop()
        print("[STOPPING CODRONE]")
        cv2.destroyAllWindows()  # shut down all windows
        sleep(1)
        drone.sendLinkDisconnect()  # disconnect from codrone
        print("[CLOSING SERIAL COMMUNICATION]")
        breakLoop = 1

    return breakLoop




def keyBoardControllerTest(cap,key):
    breakLoop = 0

    if key == ord("p"):
        sleep(1)
        print("print!!!")

    if key == ord("q"):
        print("[[[[[[[[[[[[[[[[quit]]]]]]]]]]]]]]")
        cv2.destroyAllWindows()  # shut down all windows
        breakLoop = 1

    return breakLoop
