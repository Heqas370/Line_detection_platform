from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import matplotlib.pyplot as plt
import numpy as np
from CarAPI import CarApi

car = CarApi()
right_theta = 2.3
left_theta = 0.5
angle = 0
speed = 0
lineLerr = 0
lineRerr = 0


car.setSteeringAngle(0) 
# Initialize camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

time.sleep(0.3)

while True:
    car.startContDistMeas()
    if car.getLastistance() < 2941:
        car.setSteeringAngle(0)

        car.setMotorPower(0)

    else:
        

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img = frame.array
        roiR = [img.shape[1]/2, img.shape[0]/2, img.shape[1], img.shape[0]]
        roiL = [0, img.shape[0]/2, img.shape[1]/2, img.shape[0]]
        
        #roiL = [50, 150 ,270, 200]
        #roiR = [370, 150 ,270, 200]
        #cv2.imshow("Preview", img)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 15)
        
        grayL = gray[int(roiL[1]):int(roiL[1] + roiL[3]), int(roiL[0]):int(roiL[0] + roiL[2])]
        grayR = gray[int(roiR[1]):int(roiR[1] + roiR[3]), int(roiR[0]):int(roiR[0] + roiR[2])]
        #cv2.imshow("Preview", grayL)
        
        th = cv2.adaptiveThreshold(grayL, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11 , 2)
        #mask = np.full((grayL[0], grayL[1]), 1)
        th = cv2.Canny(grayL, 75, 150)
        #cv2.imshow('test', th)
        lines = cv2.HoughLines(th, 1, np.pi/180, 20)
        
        if lines is not None:
            if lines[0][0][1] > left_theta:
                lineLerr = lines[0][0][1] - left_theta
            else:
                lineLerr = 0
                
        th = cv2.adaptiveThreshold(grayR, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11 , 2)
        th = cv2.Canny(th, 75, 150)
        lines = cv2.HoughLines(th, 1, np.pi/180, 20)
                
        if lines is not None:
            if lines[0][0][1] < right_theta:
                lineRerr = right_theta - lines[0][0][1]
            else:
                lineRerr = 0
        
           
        control = lineRerr - lineLerr
        control = (control *180/np.pi)
        angle = control *5
        #angle = control * 5
        if angle > right_theta:
            angle = right_theta*180/np.pi
        if angle < left_theta :
            angle = left_theta*180/np.pi
       
        #print(angle)
        
        if lines is not None:
            cv2.putText(img = th, text = str(angle), org = (40, 40), fontFace = cv2.FONT_HERSHEY_TRIPLEX, fontScale = 1, color  = (255), thickness = 1)
            cv2.imshow('frame', th)
        
        
        rawCapture.truncate(0)

    car.setSteeringAngle(int(angle)) 
    car.setMotorPower(speed)

    key = cv2.waitKey(1)
    if key == ord("q"):
        break
    if key == ord("z"):
        speed = 0
        
        
cv2.destroyAllWindows()
camera.close()

