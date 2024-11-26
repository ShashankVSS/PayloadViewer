import numpy as np
import cv2

captureDevice = cv2.VideoCapture(1, cv2.CAP_DSHOW) #captureDevice = camera #captureDevice = camera

while True:
    ret, frame = captureDevice.read() 

    cv2.imshow('my frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

captureDevice.release()
cv2.destroyAllWindows()