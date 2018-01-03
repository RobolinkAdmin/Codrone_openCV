from petrone.drone import *
import cv2
from keyControl import *
from detection import *

class FlappyBird:
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.throttle = 0

        #Continuous command time
        self.left_time = 0
        self.right_time = 0
        self.up_time = 0
        self.past_time = 0

        self.frame_cnt = 0
        self.right_area = 0
        self.left_area = 0

        self.battery = 0
        self.sum = 0
        self.differ = 0

        self.resize_num = 0.3
        self.batteryTimer = 0

        #--------------------#

        # PARAMETER
        self.DIFFER_PERCENT = 0.8
        self.MOVE = 5000
        # ms
        self.ROLL_TIME = 500
        self.THRO_TIME = 1000
        # drone motor
        self.ROLL = 30
        self.THRO_UP = 50
        self.THRO_DOWN = -90
        #---------------------#


    def initial_frame_cnt(self):
        self.frame_cnt,self.right_area,self.left_area = 0,0,0

    def initial_control(self):
        self.roll,self.pitch,self.yaw,self.throttle = 0,0,0,0

    def set_control(self, roll_,pitch_,yaw_,throttle_):
        self.roll = roll_
        self.pitch = pitch_
        self.yaw = yaw_
        self.throttle = throttle_

    def get_control(self):
        return self.roll,self.pitch,self.yaw,self.throttle

v = FlappyBird()

def eventUpdataBattery(data):
    v.battery = data.batteryPercent

def set_eventhandler(drone):
    drone.setEventHandler(DataType.Battery, eventUpdataBattery)

def drone_connect(drone, port_name, drone_name):
    while not drone.isConnected():
        print(" >> Connecting with drone")
        drone.close()
        if(port_name == 0):
        	drone.connect()
        else:
        	drone.connect(port_name, drone_name)
        sleep(3)

    print("Connect!")
    return drone

def divide_screen(frame,pre_frame):

    frame = cv2.resize(frame, None, fx=v.resize_num,fy=v.resize_num)
    now_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.absdiff(pre_frame,now_frame)
    frame = cv2.threshold(frame , 40, 255, cv2.THRESH_BINARY)[1]
    h,w = frame.shape
    right_frame = frame[:,int(w*0.6):]
    left_frame = frame[:,0:int(w*0.4)]
    right_num = cv2.countNonZero(right_frame)
    left_num = cv2.countNonZero(left_frame)
    return now_frame,right_num, left_num


def compare_move():
    DIFFER_THRESH = v.DIFFER_PERCENT * v.sum

    #right
    if(v.differ > DIFFER_THRESH and v.sum > v.MOVE/2):
        #check if direction is same as past
        if(v.roll > 0):
            v.right_time += v.past_time
        v.set_control(v.ROLL,0,0,0)
    #left
    elif(v.differ < -DIFFER_THRESH and v.sum > v.MOVE/2):
        if(v.roll < 0):
            v.left_time += v.past_time
        v.set_control(-v.ROLL,0,0,0)
    #up
    elif(v.sum > v.MOVE):
    	if(v.throttle == v.THRO_UP):
            v.up_time += v.past_time
        v.set_control(0,0,0,v.THRO_UP)
    #down
    else:
        v.set_control(0,0,0,v.THRO_DOWN)

def draw_for_debug(frame):
    #left and right change
    frame = cv2.flip(frame,1)
    h,w,_ = frame.shape

    #right
    if(v.roll > 0):
        cv2.rectangle(frame, (w - 10, 0), (w, h), (10, 200, 10), 3)
    #left
    elif(v.roll < 0):
        cv2.rectangle(frame, (0, 0), (10, h), (10, 200, 10), 3)
    #up
    elif(v.throttle == v.THRO_UP):
        cv2.rectangle(frame, (0, 0), (w, 10), (10, 200, 10), 3)
    #down
    elif(v.throttle == v.THRO_DOWN):
        cv2.rectangle(frame, (0,w - 10), (w,h), (10, 200, 10), 3)

    putTextonFrame(frame=frame, battPercent_= v.battery * 10, differ_= v.differ, sum_= v.sum, THRE_DIFFER= v.DIFFER_PERCENT*v.sum,
                   THRE_SUM=v.MOVE)

    return frame

def processing_capture(drone, port_name, drone_name, face_detection = 0):
    e1 = cv2.getTickCount()
    while(1):
        while drone != 0 and not drone.isConnected():
            drone_connect(drone, port_name, drone_name)
            sleep(5)

        cap = cv2.VideoCapture(0)

        if (cap.isOpened()):
            ret, frame = cap.read()
            pre_frame = cv2.resize(frame, None, fx=v.resize_num, fy=v.resize_num)
            pre_frame = cv2.cvtColor(pre_frame, cv2.COLOR_BGR2GRAY)
        else:
            print("video is not opened!")

        if(face_detection == 1):
            h, w,_ = frame.shape
            ROI = 100
            print(h, w)
            while(not face(frame, "frame",ROI)):
                cv2.rectangle(frame, (int(w / 2 - ROI),0 ), (int(w / 2 + ROI),int(h / 2)), (0, 255, 0), 3)
                cv2.imshow("frame", frame)
                cv2.waitKey(1)
                ret, frame = cap.read()
        
        if(drone != 0): drone.sendTakeOff()
        sleep(2)

        while(cap.isOpened()):
            ret, frame = cap.read()
            v.frame_cnt += 1
            pre_frame, r,l = divide_screen(frame,pre_frame)
            v.right_area += r
            v.left_area += l
            if(v.frame_cnt == 5):
                v.sum = v.right_area + v.left_area
                v.differ = v.left_area - v.right_area
                compare_move()
                if drone != 0: drone.sendControl(*v.get_control())
                # time stuff-----------------------------------------------------------------------
                e2 = cv2.getTickCount()
                timePassed = (e2 - e1) / cv2.getTickFrequency()
                e1 = e2
                fps = 1 / timePassed
                # Set the update times for each of the following the bigger the time
                # the less frequent the data will updated
                v.batteryTimer = timeCounter(timePassed, v.batteryTimer, 2)
                # ------------------------------------------------------------------------------------
                if (drone != 0 and v.batteryTimer == 0):
                    drone.sendRequest(DataType.Battery)

                v.initial_frame_cnt()
                frame = draw_for_debug(frame)
                cv2.imshow("frame",frame)
                cv2.waitKey(1)
            if (drone != 0 and keyBoardController(drone, cv2.waitKey(1) & 0xFF)):
                break
        cap.release()

def test(mode = 0, port_name = 0, drone_name = 0):
    if mode == 1: drone = Drone()
    else: drone = 0
    processing_capture(drone, port_name, drone_name, face_detection=1)

    try:
        pass
    except:
        if mode == 1:
            drone.sendStop()
            drone.sendLinkDisconnect()

if __name__ == "__main__":
    test(mode = 0)