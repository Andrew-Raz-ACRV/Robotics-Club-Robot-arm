"""
Morphological Processing example in the QUTRC Picking Challenge
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

#-----------------------------------------------------------------------------
#Red Thresholding
lower_bounds = np.array([0,50,50])
upper_bounds = np.array([15,255,255])
red_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

#Red's Morthological Processing: 
#https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
kernel = np.ones((15,15),np.uint8) #a matrix 15 by 15 ones
red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel) #Closing (fills small holes)
red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel) #Opening (removes noise in threshold)
red_mask = cv2.dilate(red_mask,kernel,iterations = 3) #Dilate white thresholds 3 times
red_mask = cv2.erode(red_mask,kernel,iterations = 3) #Erosion of the blobs 3 times
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
#Yellow Threshold
lower_bounds = np.array([20,50,50])
upper_bounds = np.array([30,255,255])
yellow_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

#Yellow's Morthological Processing
yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel) #Remove Noise
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
#Green Threshold
lower_bounds = np.array([30,50,30])
upper_bounds = np.array([90,255,255])
green_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

#Green's Morthological Processing
green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel) #Remove Noise
green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel) #Fill in small holes
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
#Blue Threshold
lower_bounds = np.array([90,50,50])
upper_bounds = np.array([150,255,255])
blue_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

#Blue's Morphological Processing
blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel) #Remove Noise
blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel) #Fill in small holes
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