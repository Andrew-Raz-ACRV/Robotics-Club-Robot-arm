"""
Blob Detection for the 4 markers in the QUTRC Picking Challenge
written by Andrew Razjigaev 2018
"""

import numpy as np
import cv2

#Read Image:
imageBGR = cv2.imread('test.jpg') #be aware that cv2 reads colours in BGR
#Convert to HSV
hsv = cv2.cvtColor(imageBGR, cv2.COLOR_BGR2HSV)


#------------------Setting up blob detection------------------------#
# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()
 
# Change thresholds - accept all white blobs
params.minThreshold = 0;
params.maxThreshold = 255;

# Filter by Area.
params.filterByArea = False
params.minArea = 1500 
 
# Filter by Circularity
params.filterByCircularity = False
params.minCircularity = 0.1
 
# Filter by Convexity
params.filterByConvexity = False
params.minConvexity = 0.87
 
# Filter by Inertia
params.filterByInertia = False
params.minInertiaRatio = 0.01
 
# Set up the blob detector with parameters.
detector = cv2.SimpleBlobDetector_create(params)

#------------------Setting up blob detection------------------------#





#Threshold bounds note its (h,s,v) so 100<h<140, 50<s<255, 50<v<255
#Note Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255]

#-----------------------------------------------------------------------------
#Red Thresholding
lower_bounds = np.array([0,50,50])
upper_bounds = np.array([15,255,255])
red_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

#Red's Morthological Processing: 
#https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
kernel = np.ones((20,20),np.uint8) #a matrix 20 by 20 ones
red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel) #Closing (fills small holes)
red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel) #Opening (removes noise in threshold)
red_mask = cv2.dilate(red_mask,kernel,iterations = 3) #Dilate white thresholds 3 times
red_mask = cv2.erode(red_mask,kernel,iterations = 3) #Erosion of the blobs 3 times

#Red's Blob detection
# Detect blobs.
red_mask = cv2.bitwise_not(red_mask) #for some reason you may need to invert black and white https://stackoverflow.com/questions/39083360/why-cant-i-do-blob-detection-on-this-binary-image
keypoints = detector.detect(red_mask)
red_mask = cv2.bitwise_not(red_mask) #Return the colours to normal

#Check no block detection case:
n = len(keypoints)

if n is 0:
    print('No red blobs detected')
else:
    #Label the red blobs found
    for keypoint in keypoints:
        x = keypoint.pt[0]
        y = keypoint.pt[1]
        s = keypoint.size
        #Threshold our labels by size
        if s > 225:
            cv2.putText(red_mask, 'Big Red Circle', (int(round(x))+150,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
        elif s > 100:
            cv2.putText(red_mask, 'Small Red Circle', (int(round(x))+150,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(red_mask, 'Red Noise', (int(round(x))+150,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
     
# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
red_mask = cv2.drawKeypoints(red_mask, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 


#-----------------------------------------------------------------------------





#-----------------------------------------------------------------------------
#Yellow Threshold
lower_bounds = np.array([20,50,50])
upper_bounds = np.array([30,255,255])
yellow_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

#Yellow's Morthological Processing
yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel) #Remove Noise
yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel) #Remove Noise
yellow_mask = cv2.dilate(yellow_mask,kernel,iterations = 3) #Dilate white thresholds 3 times
yellow_mask = cv2.erode(yellow_mask,kernel,iterations = 3) #Erosion of the blobs 3 times



#Yellow's Blob detection
# Detect blobs.
yellow_mask = cv2.bitwise_not(yellow_mask) #for some reason you may need to invert black and white https://stackoverflow.com/questions/39083360/why-cant-i-do-blob-detection-on-this-binary-image
keypoints = detector.detect(yellow_mask)
yellow_mask = cv2.bitwise_not(yellow_mask) #Return the colours to normal

#Check no block detection case:
n = len(keypoints)

if n is 0:
    print('No yellow blobs detected')
else:
    #Label the blobs found
    for keypoint in keypoints:
        x = keypoint.pt[0]
        y = keypoint.pt[1]
        s = keypoint.size
        #Threshold our labels by size
        if s > 225:
            cv2.putText(yellow_mask, 'Big Yellow Circle', (int(round(x))+150,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
        elif s > 100:
            cv2.putText(yellow_mask, 'Small Yellow Circle', (int(round(x))+150,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(yellow_mask, 'Yellow Noise', (int(round(x))+150,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
     
# Draw detected blobs as circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
yellow_mask = cv2.drawKeypoints(yellow_mask, keypoints, np.array([]), (0,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
#-----------------------------------------------------------------------------




#-----------------------------------------------------------------------------
#Green Threshold
lower_bounds = np.array([30,50,30])
upper_bounds = np.array([90,255,255])
green_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

#Green's Morthological Processing
green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel) #Remove Noise
green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel) #Fill in small holes
green_mask = cv2.dilate(green_mask,kernel,iterations = 3) #Dilate white thresholds 3 times
green_mask = cv2.erode(green_mask,kernel,iterations = 3) #Erosion of the blobs 3 times


#Green's Blob detection
# Detect blobs.
green_mask = cv2.bitwise_not(green_mask) #for some reason you may need to invert black and white https://stackoverflow.com/questions/39083360/why-cant-i-do-blob-detection-on-this-binary-image
keypoints = detector.detect(green_mask)
green_mask = cv2.bitwise_not(green_mask) #Return the colours to normal


#Check no block detection case:
n = len(keypoints)

if n is 0:
    print('No green blobs detected')
else:
    #Label the blobs found
    for keypoint in keypoints:
        x = keypoint.pt[0]
        y = keypoint.pt[1]
        s = keypoint.size
        #Threshold our labels by size
        if s > 225:
            cv2.putText(green_mask, 'Big green Circle', (int(round(x))+150,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
        elif s > 100:
            cv2.putText(green_mask, 'Small green Circle', (int(round(x))+150,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(green_mask, 'green Noise', (int(round(x))+150,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
     
# Draw detected blobs as circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
green_mask = cv2.drawKeypoints(green_mask, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
#-----------------------------------------------------------------------------




#-----------------------------------------------------------------------------
#Blue Threshold
lower_bounds = np.array([90,50,50])
upper_bounds = np.array([150,255,255])
blue_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

#Blue's Morphological Processing
blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel) #Remove Noise
blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel) #Fill in small holes
blue_mask = cv2.dilate(blue_mask,kernel,iterations = 3) #Dilate white thresholds 3 times
blue_mask = cv2.erode(blue_mask,kernel,iterations = 3) #Erosion of the blobs 3 times

#Blue's Blob detection
# Detect blobs.
blue_mask = cv2.bitwise_not(blue_mask) #for some reason you may need to invert black and white https://stackoverflow.com/questions/39083360/why-cant-i-do-blob-detection-on-this-binary-image
keypoints = detector.detect(blue_mask)
blue_mask = cv2.bitwise_not(blue_mask) #Return the colours to normal

#Check no block detection case:
n = len(keypoints)

if n is 0:
    print('No blue blobs detected')
else:
    #Label the red blobs found
    for keypoint in keypoints:
        x = keypoint.pt[0]
        y = keypoint.pt[1]
        s = keypoint.size
        #Threshold our labels by size
        if s > 225:
            cv2.putText(blue_mask, 'Big blue Circle', (int(round(x))+150,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
        elif s > 100:
            cv2.putText(blue_mask, 'Small blue Circle', (int(round(x))+150,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(blue_mask, 'blue Noise', (int(round(x))+150,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
     
# Draw detected blobs as circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
blue_mask = cv2.drawKeypoints(blue_mask, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
#-----------------------------------------------------------------------------





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