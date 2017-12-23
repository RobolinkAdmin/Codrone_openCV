from time import sleep
import cv2

def putTextonFrame(frame, fps, roll_, pitch_, yaw_, throttle_):
    font = cv2.FONT_HERSHEY_SIMPLEX

    cv2.putText(frame, "FPS", (0, 20), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, '{:04.2f}'.format(fps), (55, 20), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    cv2.putText(frame, "Throttle", (0, 110), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, '{:04.2f}'.format(throttle_), (60, 110), font, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    cv2.putText(frame, "Yaw", (0, 110 + 30), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, '{:04.2f}'.format(yaw_), (60, 110 + 30), font, 0.5, (255, 0, 255), 1, cv2.LINE_AA)

    cv2.putText(frame, "Roll", (0, 110 + 60), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, '{:04.2f}'.format(roll_), (60, 110 + 60), font, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

    cv2.putText(frame, "Pitch", (0, 110 + 90), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, '{:04.2f}'.format(pitch_), (60, 110 + 90), font, 0.5, (255, 255, 0), 1, cv2.LINE_AA)


# this method takes in the the key pressed and
# returns 0 or 1 iuf you want to break out of the loop
def keyBoardController(drone,cap,key):
    breakLoop = 0
    # THROTTLE
    # ----------------------------------------------------------
    if key == ord("w"):
        # drone.sendControl(roll_, pitch_, yaw_, throttle_)
        drone.sendControl(0, 0, 0, 50)
    # ----------------------------------------------------------
    if key == ord("s"):
        drone.sendControl(0, 0, 0, -50)
    ##YAW
    # ----------------------------------------------------------
    if key == ord("a"):
        drone.sendControl(0, 0, 50, 0)
    # ----------------------------------------------------------
    if key == ord("d"):
        drone.sendControl(0, 0, -50, 0)
    ##PITCH
    # ----------------------------------------------------------
    if key == ord("i"):
        drone.sendControl(0, 50, 0, 0)
    # ----------------------------------------------------------
    if key == ord("k"):
        drone.sendControl(0, -50, 0, 0)

    ##ROLL
    # ----------------------------------------------------------
    if key == ord("j"):
        drone.sendControl(-20, 0, 0, -10)
    # ----------------------------------------------------------
    if key == ord("l"):
        drone.sendControl(20, 0, 0, -10)

    # ----------------------------------------------------------

    ##CONNECTING
    # ----------------------------------------------------------
    if key == ord("r"):
        drone.connect()
        sleep(2)
        print("CONNECTING TO CODRONE")

    if key == ord("t"):
        drone.sendTakeOff()
        sleep(1)
        print("TAKEOFF CODRONE")

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
        cap.release()  # release video capture
        cv2.destroyAllWindows()  # shut down all windows
        sleep(1)
        drone.sendLinkDisconnect()  # disconnect from codrone
        drone.close()  # close drone
        print("[CLOSING SERIAL COMMUNICATION]")
        breakLoop = 1

    return breakLoop
