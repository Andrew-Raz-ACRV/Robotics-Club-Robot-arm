from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 

camera = PiCamera()
rawCapture = PiRGBArray(camera)

time.sleep(0.1)

camera.capture(rawCapture, format="bgr")
imageBGR = rawCapture.array

#Resize images for display
scale = 0.3;
width = int(imageBGR.shape[1]*scale); #extract shape width and scale it
height = int(imageBGR.shape[0]*scale); #extract shape height and scale it
imageBGRs = cv2.resize(imageBGR, (width,height))

#flip
imageBGRs = cv2.flip(imageBGRs, -1)

#Show image
cv2.imshow('Colour Image',imageBGRs)

cv2.waitKey(0)                 # Waits forever for user to press any key
cv2.destroyAllWindows()        # Closes displayed windows
