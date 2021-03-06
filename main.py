import numpy as np
import cv2
from time import sleep
import math
from threading import Thread

# -------------------V--OPENCV--V----------------------

cap = cv2.VideoCapture(0)
##fgbg = cv2.createBackgroundSubtractorMOG2()
tempret, initframe = cap.read()
#sleep(1)
calib = cv2.cvtColor(cap.read()[1], cv2.COLOR_BGR2GRAY)
prev_frame = cv2.cvtColor(initframe, cv2.COLOR_BGR2GRAY)

height, width = initframe.shape[:2]

fov = 75.0 # Degrees
trackball_radius = 2.0
pixel_ang_size = fov/width


font = cv2.FONT_HERSHEY_SIMPLEX

def opencv_loop():
    while(True):
        _, frame = cap.read()
        ##gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        ##res = cv2.subtract(calib, gray)

        ##_, blurskinthresh = cv2.threshold(cv2.medianBlur(res,5), 50, 255, cv2.THRESH_BINARY)

        lower_blue = np.array([70,10,75])
        upper_blue = np.array([100,255,255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        mask = cv2.medianBlur(mask, 5)
        blue = cv2.bitwise_and(frame,frame, mask= mask)
        
        ##graythresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 1)
        ##_, graythresh = cv2.threshold(gray, 115, 255, cv2.THRESH_BINARY)
        ##change = cv2.subtract(gray, prev_frame)
        ##fgmask = fgbg.apply(frame)
        ##_, fg = cv2.threshold(fgmask, 250, 255, cv2.THRESH_BINARY)
        ##_, changethresh = cv2.threshold(change, 115, 255, cv2.THRESH_BINARY)
        ##changethresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 1)
        circles = cv2.HoughCircles(cv2.medianBlur(mask, 5),cv2.HOUGH_GRADIENT,1,20,param1=1,param2=10,minRadius=5,maxRadius=0)
        try:
            if circles.any():
                circles = np.uint16(np.around(circles))
                # draw the outer circle
                cv2.circle(frame,(circles[0][0][0],circles[0][0][1]),circles[0][0][2],(0,0,255),2)
                # draw the center of the circle
                cv2.circle(frame,(circles[0][0][0],circles[0][0][1]),2,(0,0,255),3)
                # calculate positional data
                ang_size = float(circles[0][0][2]) * pixel_ang_size
                distance = trackball_radius / math.tan(math.radians(ang_size))

                cv2.putText(frame,'ang_size: ' + str(ang_size),(50,400), font, 1,(255,255,255),2,cv2.LINE_AA)
                cv2.putText(frame,'distance: ' + str(distance/100) + "m",(50,470), font, 1,(255,255,255),2,cv2.LINE_AA)
        except:
            pass
        cv2.imshow('Original',frame)
        cv2.imshow('Blue', blue)
        ##cv2.imshow('Arms', blurskinthresh)
        ##cv2.imshow('Foreground', res)
        ##cv2.imshow('Foreground', fg)
        ##cv2.imshow('Movement Threshold', changethresh)
        
        ##prev_frame = gray
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

# -------------------V-- THREADING MANAGEMENT --V------------------
    
opencv_loop()

cap.release()
cv2.destroyAllWindows()
