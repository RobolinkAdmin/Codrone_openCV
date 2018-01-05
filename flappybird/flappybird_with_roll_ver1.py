from petrone.drone import *
import cv2
import threading
from keyControl import *
from detection import *
import numpy


class FlappyBird:
    def __init__(self):

        self.quit = 0

        self.ROI = 100

        self.face_image = cv2.imread("face_img.png")
        if (self.face_image is None): print("No Face image")
        self.face_image = cv2.resize(self.face_image, (self.ROI * 2, self.ROI * 2))

        self.bird_image = cv2.imread("bird_img.png", -1)
        self.bird_mask = self.bird_image[:, :, 3:]
        self.bird_image = cv2.cvtColor(self.bird_image, cv2.COLOR_BGRA2BGR)
        if (self.bird_image is None): print("No Bird image")

        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.throttle = 0

        # Continuous command time
        self.left_time = 0
        self.right_time = 0
        self.up_time = 0
        self.past_time = 0

        self.frame_cnt = 0
        self.right_area = 0
        self.left_area = 0


        self.irRange = 0
        self.battery = 0

        self.sum = 0
        self.differ = 0

        self.resize_num = 0.3
        self.batteryTimer = 0
        self.irRangeTimer = 0
        # --------------------#

        # PARAMETER
        self.DIFFER_PERCENT = 0.8
        self.MOVE = 5000
        # ms
        self.ROLL_TIME = 500
        self.THRO_TIME = 1000
        # drone motor
        self.PITCH = 20
        self.ROLL = 30
        self.THRO_UP = 50
        self.THRO_DOWN = -90
        self.LOW_THRO_UP = 30
        # ---------------------#

    def initial_frame_cnt(self):
        self.frame_cnt, self.right_area, self.left_area = 0, 0, 0

    def initial_control(self):
        self.roll, self.pitch, self.yaw, self.throttle = 0, 0, 0, 0

    def set_control(self, roll_, pitch_, yaw_, throttle_):
        self.roll = roll_
        self.pitch = pitch_
        self.yaw = yaw_
        self.throttle = throttle_

    def get_control(self):
        return self.roll, self.pitch, self.yaw, self.throttle


v = FlappyBird()

def eventUpdataBattery(data):
    v.battery = data.batteryPercent

def eventUpdataRange(data):
    v.irRange = data.bottom

def set_eventhandler(drone):
    drone.setEventHandler(DataType.Battery, eventUpdataBattery)
    drone.setEventHandler(DataType.Range, eventUpdataRange)

def drone_connect(drone, port_name, drone_name):
    while not drone.isConnected():
        print(" >> Connecting with drone")
        drone.close()
        if (port_name == 0):
            drone.connect()
        else:
            drone.connect(port_name, drone_name)
        sleep(3)

    print("Connect!")
    return drone


def divide_screen(frame, pre_frame):
    frame = cv2.resize(frame, None, fx=v.resize_num, fy=v.resize_num)
    now_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.absdiff(pre_frame, now_frame)
    frame = cv2.threshold(frame, 40, 255, cv2.THRESH_BINARY)[1]
    h, w = frame.shape
    right_frame = frame[:, int(w * 0.6):]
    left_frame = frame[:, 0:int(w * 0.4)]
    right_num = cv2.countNonZero(right_frame)
    left_num = cv2.countNonZero(left_frame)
    return now_frame, right_num, left_num


def compare_move(roll_flag = 1):
    DIFFER_THRESH = v.DIFFER_PERCENT * v.sum
    if(roll_flag):
        pitch = v.PITCH
    else:
        pitch = 0

    # right
    if (roll_flag and v.differ > DIFFER_THRESH and v.sum > v.MOVE / 2):
        # check if direction is same as past
        if (v.roll > 0):
            v.right_time += v.past_time
        v.set_control(v.ROLL, 0, 0, 0)
    # left
    elif (roll_flag and v.differ < -DIFFER_THRESH and v.sum > v.MOVE / 2):
        if (v.roll < 0):
            v.left_time += v.past_time
        v.set_control(-v.ROLL, 0, 0, 0)
    # up
    elif (v.sum > v.MOVE):
        if (v.throttle == v.THRO_UP):
            v.up_time += v.past_time
        if (roll_flag):
            v.set_control(pitch, 0, 0, v.THRO_UP)
        else:
            v.set_control(0,0,0,v.LOW_THRO_UP)
    # down
    else:
        v.set_control(pitch, 0, 0, v.THRO_DOWN)


def draw_for_debug(frame,roll_version = 1):
    # left and right change
    frame = cv2.flip(frame, 1)
    h, w, _ = frame.shape

    # right
    if (roll_version and v.roll > 0):
        cv2.rectangle(frame, (w - 10, 0), (w, h), (200, 200, 200), 5)
    # left
    elif (roll_version and v.roll < 0):
        cv2.rectangle(frame, (0, 0), (10, h), (200, 200, 200), 5)
    # up
    elif (v.throttle > v.LOW_THRO_UP):
        cv2.rectangle(frame, (0, 0), (w, 10), (200, 200, 200), 5)
    # down
    elif (v.throttle == v.THRO_DOWN):
        cv2.rectangle(frame, (0, w - 10), (w, h), (200, 200, 200), 5)

    putTextonFrame(frame=frame, battPercent_=v.battery * 10, differ_=v.differ, sum_=v.sum,
                   THRE_DIFFER=v.DIFFER_PERCENT * v.sum,
                   THRE_SUM=v.MOVE)
    return frame


def processing_capture(drone, port_name, drone_name, roll_version = 1, face_detection=0):
    e1 = cv2.getTickCount()
    while (quit == 0):
        while not drone.isConnected():
            drone_connect(drone, port_name, drone_name)
            sleep(5)
            if (keyBoardController(drone, cv2.waitKey(1) & 0xFF)):
                break
        print("connect")
        cap = cv2.VideoCapture(0)

        if (cap.isOpened()):
            ret, frame = cap.read()
            frame = cv2.flip(frame, 1)
            pre_frame = cv2.resize(frame, None, fx=v.resize_num, fy=v.resize_num)
            pre_frame = cv2.cvtColor(pre_frame, cv2.COLOR_BGR2GRAY)
        else:
            print("video is not opened!")

        if (face_detection == 1):
            h, w, _ = frame.shape
            w_half = int(w / 2)
            h_quar = int(h / 4)
            if (h_quar < v.ROI): h_quar += (h_quar - v.ROI)
            if (w_half < v.ROI): w_half += (w_half - v.ROI)
            while (not face(frame, "frame", v.ROI, w_half, h_quar)):
                # cv2.rectangle(frame, (w_half - v.ROI, h_quar - v.ROI), (w_half + v.ROI, h_quar + v.ROI), (230, 230, 230), 10, cv2.LINE_AA)
                if (v.face_image is not None):
                    frame[h_quar - v.ROI: h_quar + v.ROI, w_half - v.ROI:w_half + v.ROI] = cv2.bitwise_and(v.face_image,
                                                                                                           frame[
                                                                                                           h_quar - v.ROI: h_quar + v.ROI,
                                                                                                           w_half - v.ROI:w_half + v.ROI])
                cv2.imshow("frame", frame)
                cv2.waitKey(1)
                ret, frame = cap.read()
                frame = cv2.flip(frame, 1)
                if (keyBoardController(drone, cv2.waitKey(1) & 0xFF)):
                    break

            if (v.bird_image is not None):
                v.bird_image = cv2.resize(v.bird_image, (w, h))
                v.bird_mask = cv2.resize(v.bird_mask, (w, h))
            img1 = cv2.bitwise_and(frame, frame, mask=~v.bird_mask)
            img2 = cv2.bitwise_and(v.bird_image, v.bird_image, mask=v.bird_mask)
            frame = cv2.add(img1, img2)

            cv2.imshow("frame", frame)
            cv2.waitKey(1)

        if (drone.isConnected()):
            drone.sendTakeOff()
        sleep(3)

        while (cap.isOpened()):
            ret, frame = cap.read()
            v.frame_cnt += 1
            pre_frame, r, l = divide_screen(frame, pre_frame)
            v.right_area += r
            v.left_area += l
            if (v.frame_cnt == 5):
                v.sum = v.right_area + v.left_area
                v.differ = v.left_area - v.right_area
                compare_move(roll_version)
                print(v.get_control())
                if drone.isConnected(): drone.sendControl(*v.get_control())
                # time stuff-----------------------------------------------------------------------
                e2 = cv2.getTickCount()
                timePassed = (e2 - e1) / cv2.getTickFrequency()
                print(timePassed, v.batteryTimer, v.irRangeTimer)
                e1 = e2
                fps = 1 / timePassed
                # Set the update times for each of the following the bigger the time
                # the less frequent the data will updated
                v.batteryTimer = timeCounter(timePassed, v.batteryTimer, 5)
                v.irRangeTimer = timeCounter(timePassed, v.irRangeTimer, 1)
                # ------------------------------------------------------------------------------------
                if (drone.isConnected() and v.irRangeTimer == 0):
                    drone.sendRequest(DataType.Range)
                elif (drone.isConnected() and v.batteryTimer == 0):
                    drone.sendRequest(DataType.Battery)
                print(v.battery, v.irRange)
                v.initial_frame_cnt()
                frame = draw_for_debug(frame,roll_version)
                cv2.imshow("frame", frame)
                cv2.waitKey(1)
            key = keyBoardController(drone, cv2.waitKey(1) & 0xFF)
            if (key or v.irRange > 2000):
                if(key == 2): v.quit = 1
                break
        cap.release()
        drone.close()



def test(roll_version, port_name=None, drone_name=None):
    drone = Drone()
    set_eventhandler(drone)

    processing_capture(drone,port_name, drone_name, roll_version, face_detection=1)

if __name__ == "__main__":
    test(roll_version = 0, port_name="COM18", drone_name="PETRONE 7463")