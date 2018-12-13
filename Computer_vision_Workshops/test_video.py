# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
 
# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
        imageBGR = frame.array
	
	#flip
        imageBGR = cv2.flip(imageBGR, -1)
        
        #Convert to HSV
        hsv = cv2.cvtColor(imageBGR, cv2.COLOR_BGR2HSV)

        #Red Thresholding
        lower_bounds = np.array([0,50,50])
        upper_bounds = np.array([10,255,255])
        red_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

        #Yellow Threshold
        lower_bounds = np.array([20,50,50])
        upper_bounds = np.array([30,255,255])
        yellow_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)
        
        #Green Threshold
        lower_bounds = np.array([30,50,10])
        upper_bounds = np.array([90,255,255])
        green_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)
        
        #Blue Threshold
        lower_bounds = np.array([90,50,50])
        upper_bounds = np.array([150,255,255])
        blue_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)


        #Show frame
        #cv2.imshow('Colour Image',imageBGRs)
        #cv2.imshow('Red Thresholding',red_mask)
        #cv2.imshow('Yellow Thresholding',yellow_mask) 
        #cv2.imshow('Green Thresholding',green_mask) 
        cv2.imshow('Blue Thresholding',blue_mask) 

        key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
        rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
        if key == ord("q"):
                break
