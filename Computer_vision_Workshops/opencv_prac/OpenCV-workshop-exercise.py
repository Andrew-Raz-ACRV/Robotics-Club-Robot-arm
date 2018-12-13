"""
HSV Thresholding for the 4 markers in the QUTRC Picking Challenge
Exercise for blob detection and morphological image processing
written by Andrew Razjigaev 2018
"""

import numpy as np
import cv2

#Read Image:
imageBGR = cv2.imread('test.jpg') #be aware that cv2 reads colours in BGR
#Convert to HSV
hsv = cv2.cvtColor(imageBGR, cv2.COLOR_BGR2HSV)

#Threshold bounds note its (h,s,v) so 100<h<140, 50<s<255, 50<v<255
#Note Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255]

#Red Thresholding
lower_bounds = np.array([0,50,50])
upper_bounds = np.array([15,255,255])
red_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

#TODO: Morhological processing

#Use this website for more information:
##https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html



#e.g. 

#kernel = np.ones((20,20),np.uint8) #a matrix 20 by 20 ones

#red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel) #Closing (fills small holes)
#red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel) #Opening (removes noise in threshold)
#red_mask = cv2.dilate(red_mask,kernel,iterations = 3) #Dilate white thresholds 3 times
#red_mask = cv2.erode(red_mask,kernel,iterations = 3) #Erosion of the blobs 3 times



#TODO Blob detection

#Hint search about this function:
#detector = cv2.SimpleBlobDetector_create(params)

## Example of doing it: Detect blobs.
#red_mask = cv2.bitwise_not(red_mask) 
##for some reason you may need to invert black and white https://stackoverflow.com/questions/39083360/why-cant-i-do-blob-detection-on-this-binary-image
#keypoints = detector.detect(red_mask)
#red_mask = cv2.bitwise_not(red_mask) #Return the colours to normal


#Yellow Threshold
lower_bounds = np.array([20,50,50])
upper_bounds = np.array([30,255,255])
yellow_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

#Todo morphological processing and blob detection

#Green Threshold
lower_bounds = np.array([30,50,30])
upper_bounds = np.array([90,255,255])
green_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

#Todo morphological processing and blob detection

#Blue Threshold
lower_bounds = np.array([90,50,50])
upper_bounds = np.array([150,255,255])
blue_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

#Todo morphological processing and blob detection

#-----------------------------------------------------------------------------
#Resize images for display
scale = 0.3;
width = int(imageBGR.shape[1]*scale); #extract shape width and scale it
height = int(imageBGR.shape[0]*scale); #extract shape height and scale it
imageBGRs = cv2.resize(imageBGR, (width,height))

#Threshold images
red_masks = cv2.resize(red_mask, (width,height)) 
yellow_masks = cv2.resize(yellow_mask, (width,height)) 
green_masks = cv2.resize(green_mask, (width,height)) 
blue_masks = cv2.resize(blue_mask, (width,height)) 


#Show image
cv2.imshow('Colour Image',imageBGRs)
cv2.imshow('Red Thresholding',red_masks)  
cv2.imshow('Yellow Thresholding',yellow_masks) 
cv2.imshow('Green Thresholding',green_masks) 
cv2.imshow('Blue Thresholding',blue_masks) 

cv2.waitKey(0)                 # Waits forever for user to press any key
cv2.destroyAllWindows()        # Closes displayed windows
