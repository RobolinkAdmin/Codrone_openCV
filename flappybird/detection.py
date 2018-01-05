import cv2

face_detector = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
def face(image, window, ROI, W, H):
    roi = image[H - ROI :H + ROI, W - ROI:W + ROI]
    faces = face_detector.detectMultiScale(roi, 1.3, 5, 10)

    if(len(faces) == 0): return False
    else: return True
"""
    for (x, y, w, h) in faces:
        # Crop the image frame into rectangle
        for i in range(0,10,2):
            cv2.rectangle(image, (W - ROI + x+i, H-ROI + y+i), (W - ROI + x + w-i, H-ROI+y+h-i), (255 - 10*i, 255 - 10*i, 255 - 10*i), 1)
            #cv2.rectangle(image, (W_half - ROI, 0), (W_half + ROI, H_half), (255-i, 255-i, 255-i), 1)

        cv2.imshow(window, image)
        cv2.waitKey(100)
        return True
"""