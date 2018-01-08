from petrone_v2.drone import *
import cv2
from serial.tools.list_ports import comports
from keyControl import *
from detection import *


class FlappyBird:
    def __init__(self):
        # --------------------#
        # PARAMETER
        self.ROI = 100

        self.resize_num = 0.3

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

        self.drone = Drone()
        self.cap = cv2.VideoCapture(0)
        self.frame_name = "frame"

        self.quit = 0

        self.face_mask = cv2.imread("face_img.png", -1)[:, :, 3:]
        self.face_image = cv2.imread("face_img.png")
        self.start_mask = cv2.imread("start_img.png", -1)[:, :, 3:]
        self.start_image = cv2.imread("start_img.png")

        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.throttle = 0

        self.irRange = 0
        self.battery = 0

        # Continuous command time
        self.left_time = 0
        self.right_time = 0
        self.up_time = 0
        self.past_time = 0

        self.frame_cnt = 0
        self.right_area = 0
        self.left_area = 0

        self.batteryTimer = 0
        self.irRangeTimer = 0

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

    # EVENT HANDLER
    def eventUpdataBattery(self, data):
        self.battery = data.batteryPercent

    def eventUpdataRange(self, data):
        self.irRange = data.bottom

    def set_eventhandler(self):
        self.drone.setEventHandler(DataType.Battery, self.eventUpdataBattery)
        self.drone.setEventHandler(DataType.Range, self.eventUpdataRange)


    """
    Divide the screen with left side and right side for comparing each movement
    """

    def divide_screen(self, frame, pre_frame):

        frame = cv2.resize(frame, None, fx=self.resize_num, fy=self.resize_num)
        now_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.absdiff(pre_frame, now_frame)
        frame = cv2.threshold(frame, 40, 255, cv2.THRESH_BINARY)[1]
        h, w = frame.shape
        right_frame = frame[:, int(w * 0.6):]
        left_frame = frame[:, 0:int(w * 0.4)]
        right_num = cv2.countNonZero(right_frame)
        left_num = cv2.countNonZero(left_frame)
        ee1 = cv2.getTickCount()

        return now_frame, right_num, left_num

    def compare_move(self, sum, differ, roll_flag=1):
        DIFFER_THRESH = self.DIFFER_PERCENT * sum

        # right
        # go left
        if (roll_flag and differ > DIFFER_THRESH and sum > self.MOVE / 2):
            # check if direction is same as past
            if (self.roll > 0):
                self.right_time += self.past_time
            self.set_control(-self.ROLL, 0, 0, 0)
        # left
        # go right
        elif (roll_flag and differ < -DIFFER_THRESH and sum > self.MOVE / 2):
            if (self.roll < 0):
                self.left_time += self.past_time
            self.set_control(self.ROLL, 0, 0, 0)
        # up
        elif (sum > self.MOVE):
            if (self.throttle == self.THRO_UP):
                self.up_time += self.past_time
            if (roll_flag):
                self.set_control(0, self.PITCH, 0, self.THRO_UP)
            else:
                self.set_control(0, 0, 0, self.LOW_THRO_UP)
        # down
        else:
            self.set_control(0, 0, 0, self.THRO_DOWN)

    """
    Draw the direction for drone
    """

    def draw_for_debug(self, frame, sum, differ, roll_version=1):
        # left and right change
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape

        # right
        if (roll_version and self.roll < 0):
            cv2.rectangle(frame, (w - 10, 0), (w, h), (200, 200, 200), 5)
        # left
        elif (roll_version and self.roll > 0):
            cv2.rectangle(frame, (0, 0), (10, h), (200, 200, 200), 5)
        # up
        elif (self.throttle > self.LOW_THRO_UP):
            cv2.rectangle(frame, (0, 0), (w, 10), (200, 200, 200), 5)
        # down
        elif (self.throttle == self.THRO_DOWN):
            cv2.rectangle(frame, (0, w - 10), (w, h), (200, 200, 200), 5)

        putTextonFrame(frame=frame, battPercent_=self.battery * 10, differ_=differ, sum_=sum,
                       THRE_DIFFER=self.DIFFER_PERCENT * sum,
                       THRE_SUM=self.MOVE)
        return frame

    """
    Draw image on frame.
    img_w,img_h : image width,image height
    center_x, center_y : image center location on frame
    """

    def draw_img(self, frame, image, mask, img_w, img_h, center_x, center_y):

        x = center_x - int(img_w / 2)
        y = center_y - int(img_h / 2)
        frame_copy = frame.copy()
        frame_mask = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)
        ret, frame_mask = cv2.threshold(frame_mask, 1, 255, cv2.THRESH_BINARY)

        img = cv2.resize(image, (img_h, img_w))
        img_mask = cv2.resize(mask, (img_h, img_w))

        frame_copy[y:y + img_h, x:x + img_w] = img
        frame_mask[y:y + img_h, x:x + img_w] = img_mask

        img1 = cv2.bitwise_and(frame, frame, mask=~frame_mask)
        img2 = cv2.bitwise_and(frame_copy, frame_copy, mask=frame_mask)
        return cv2.add(img1, img2)

    """
    Using face detection for start and draw the image
    ROI center location : height = 1/4 width = 1/2   fixed.
    """

    def face_detect(self, frame):
        h, w, _ = frame.shape
        w_half = int(w / 2)
        h_half = int(h / 2)
        h_quar = int(h / 4)
        if (h_quar < self.ROI or w_half < self.ROI): print("ROI IS TOO BIG!")

        while (not face(frame, self.frame_name, self.ROI, w_half, h_quar)):
            frame = self.draw_img(frame, self.face_image, self.face_mask, self.ROI * 2, self.ROI * 2, w_half, h_quar)
            cv2.imshow(self.frame_name, frame)
            cv2.waitKey(1)
            ret, frame = self.cap.read()
            frame = cv2.flip(frame, 1)
            key = keyBoardController(self.drone, cv2.waitKey(1) & 0xFF)
            if (key):
                if (key is 2): self.quit = 1
                break
        # Show the starting image
        frame = self.draw_img(frame, self.start_image, self.start_mask, w_half, w_half, w_half, h_half)
        cv2.imshow(self.frame_name, frame)
        cv2.waitKey(1)

    def run(self, port_name=None, drone_name=None, roll_version=1, face_detection=0):
        self.set_eventhandler()

        while (self.quit is 0):
            e1 = cv2.getTickCount()
            ### Port open
            self.drone.open()
            #self.drone.open(port,name)

            ### vedio capture and save to previous frame
            if (self.cap.isOpened()):
                ret, frame = self.cap.read()
                frame = cv2.flip(frame, 1)
                pre_frame = cv2.resize(frame, None, fx=self.resize_num, fy=self.resize_num)
                pre_frame = cv2.cvtColor(pre_frame, cv2.COLOR_BGR2GRAY)
            else:
                print("video is not opened!")

            cv2.namedWindow(self.frame_name, cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty(self.frame_name, cv2.WND_PROP_VISIBLE, cv2.WINDOW_FULLSCREEN)
            cv2.imshow(self.frame_name, frame)
            cv2.waitKey(1)
            ### Start with face detection
            if (face_detection == 1):
                self.face_detect(frame)

            ### Start with takeoff

            self.drone.sendTakeOff()
            sleep(3)

            while (self.cap.isOpened() and self.quit is 0):
                ret, frame = self.cap.read()
                self.frame_cnt += 1
                pre_frame, r, l = self.divide_screen(frame, pre_frame)
                self.right_area += r
                self.left_area += l

                ### Send the commands every 5 frame
                if (self.frame_cnt == 5):
                    sum = self.right_area + self.left_area
                    differ = self.left_area - self.right_area
                    self.initial_frame_cnt()

                    self.compare_move(sum, differ, roll_version)
                    self.drone.sendControl(*self.get_control())

                    # time stuff-----------------------------------------------------------------------
                    e2 = cv2.getTickCount()
                    timePassed = (e2 - e1) / cv2.getTickFrequency()
                    e1 = e2
                    fps = 1 / timePassed

                    # Set the update times for each of the following the bigger the time
                    # the less frequent the data will updated
                    self.batteryTimer = timeCounter(timePassed, self.batteryTimer, 1)
                    # self.irRangeTimer = timeCounter(timePassed, self.irRangeTimer, 4)
                    if (self.batteryTimer == 0):
                        self.drone.sendRequest(DeviceType.Broadcasting, DataType.Battery)
                        sleep(0.03)
                    
                    # We could check battery problem
                    if (self.battery != 0 and self.battery < 10): print(self.battery)
                    # ------------------------------------------------------------------------------------

                    frame = self.draw_for_debug(frame, sum, differ, roll_version)
                    cv2.imshow(self.frame_name, frame)
                    cv2.waitKey(1)

                key = keyBoardController(self.drone, cv2.waitKey(1) & 0xFF)
                if (key):
                    if (key == 2): self.quit = 1
                    break
        self.cap.release()
        self.drone.close()


if __name__ == "__main__":
    #FlappyBird().run(roll_version=1, face_detection=1)
    FlappyBird().run(roll_version = 1, face_detection=1, port_name="COM19", drone_name="PETRONE 8097")