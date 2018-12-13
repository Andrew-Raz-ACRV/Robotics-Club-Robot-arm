# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

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
 
# Create a detector with the parameters
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3 :
    detector = cv2.SimpleBlobDetector(params)
else : 
    detector = cv2.SimpleBlobDetector_create(params)

#------------------Setting up blob detection------------------------#

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
 
# allow the camera to warmup
time.sleep(0.1)

#Bool for the escape camera feed
got_target = False
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    if (got_target==True):
        break
    else:
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
        imageBGR = frame.array
	
	#flip
        imageBGR = cv2.flip(imageBGR, -1)
        
        #Convert to HSV
        hsv = cv2.cvtColor(imageBGR, cv2.COLOR_BGR2HSV)


        #-----------------------------------------------------------------------------
        #Red Thresholding
        lower_bounds = np.array([0,50,50])
        upper_bounds = np.array([10,255,255])
        red_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

        #Red's Morthological Processing: 
        #https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
        kernel = np.ones((15,15),np.uint8) #a matrix 15 by 15 ones
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

        #Initialise ouputs
        red_target = [0,0]
        red_dot = [0,0]

        if n is 0:
            print('No red blobs detected')
        else:
            #Label the red blobs found
            for keypoint in keypoints:
                x = keypoint.pt[0]
                y = keypoint.pt[1]
                s = keypoint.size
                #print(s)
                #Threshold our labels by size
                if s > 19:
                    cv2.putText(red_mask, 'Big Red Circle', (int(round(x))+50,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
                    red_target = [x,y]
                elif s > 12:
                    cv2.putText(red_mask, 'Small Red Circle', (int(round(x))+50,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
                    red_dot = [x,y]
                else:
                    cv2.putText(red_mask, 'Red Noise', (int(round(x))+50,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
             
 
        #-----------------------------------------------------------------------------





        #-----------------------------------------------------------------------------
        #Yellow Threshold
        lower_bounds = np.array([20,50,50])
        upper_bounds = np.array([30,255,255])
        yellow_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

        #Yellow's Morthological Processing
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel) #Remove Noise

        #Yellow's Blob detection
        # Detect blobs.
        yellow_mask = cv2.bitwise_not(yellow_mask) #for some reason you may need to invert black and white https://stackoverflow.com/questions/39083360/why-cant-i-do-blob-detection-on-this-binary-image
        keypoints = detector.detect(yellow_mask)
        yellow_mask = cv2.bitwise_not(yellow_mask) #Return the colours to normal

        #Check no block detection case:
        n = len(keypoints)

        #Initialise ouputs
        yellow_target = [0,0]
        yellow_dot = [0,0]

        if n is 0:
            print('No yellow blobs detected')
        else:
            #Label the blobs found
            for keypoint in keypoints:
                x = keypoint.pt[0]
                y = keypoint.pt[1]
                s = keypoint.size
                #Threshold our labels by size
                if s > 19:
                    cv2.putText(yellow_mask, 'Big Yellow Circle', (int(round(x))+50,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
                    yellow_target = [x,y]
                elif s > 12:
                    cv2.putText(yellow_mask, 'Small Yellow Circle', (int(round(x))+50,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
                    yellow_dot = [x,y]
                else:
                    cv2.putText(yellow_mask, 'Yellow Noise', (int(round(x))+50,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
             

        #-----------------------------------------------------------------------------




        #-----------------------------------------------------------------------------
        #Green Threshold
        lower_bounds = np.array([30,50,10])
        upper_bounds = np.array([90,255,255])
        green_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

        #Green's Morthological Processing
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel) #Remove Noise
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel) #Fill in small holes

        #Green's Blob detection
        # Detect blobs.
        green_mask = cv2.bitwise_not(green_mask) #for some reason you may need to invert black and white https://stackoverflow.com/questions/39083360/why-cant-i-do-blob-detection-on-this-binary-image
        keypoints = detector.detect(green_mask)
        green_mask = cv2.bitwise_not(green_mask) #Return the colours to normal

        #Check no block detection case:
        n = len(keypoints)

        #Initialise ouputs
        green_target = [0,0]
        green_dot = [0,0]

        if n is 0:
            print('No green blobs detected')
        else:
            #Label the blobs found
            for keypoint in keypoints:
                x = keypoint.pt[0]
                y = keypoint.pt[1]
                s = keypoint.size
                #Threshold our labels by size
                if s > 19:
                    cv2.putText(green_mask, 'Big green Circle', (int(round(x))+50,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
                    green_target = [x,y]
                elif s > 10:
                    cv2.putText(green_mask, 'Small green Circle', (int(round(x))+50,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
                    green_dot = [x,y]
                else:
                    cv2.putText(green_mask, 'green Noise', (int(round(x))+50,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
 
        #-----------------------------------------------------------------------------




        #-----------------------------------------------------------------------------
        #Blue Threshold
        lower_bounds = np.array([90,50,50])
        upper_bounds = np.array([150,255,255])
        blue_mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

        #Blue's Morphological Processing
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel) #Remove Noise
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel) #Fill in small holes

        #Blue's Blob detection
        # Detect blobs.
        blue_mask = cv2.bitwise_not(blue_mask) #for some reason you may need to invert black and white https://stackoverflow.com/questions/39083360/why-cant-i-do-blob-detection-on-this-binary-image
        keypoints = detector.detect(blue_mask)
        blue_mask = cv2.bitwise_not(blue_mask) #Return the colours to normal

        #Check no block detection case:
        n = len(keypoints)

        #Initialise ouputs
        blue_target = [0,0]
        blue_dot = [0,0]

        if n is 0:
            print('No blue blobs detected')
        else:
            #Label the red blobs found
            for keypoint in keypoints:
                x = keypoint.pt[0]
                y = keypoint.pt[1]
                s = keypoint.size
                #Threshold our labels by size
                if s > 19:
                    cv2.putText(blue_mask, 'Big blue Circle', (int(round(x))+50,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
                    blue_target = [x,y]
                elif s > 12:
                    cv2.putText(blue_mask, 'Small blue Circle', (int(round(x))+50,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
                    blue_dot = [x,y]
                else:
                    cv2.putText(blue_mask, 'blue Noise', (int(round(x))+50,int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
             
 
        #-----------------------------------------------------------------------------
        
        #-----------------------------------------------------------------------------
        #Homography

        #The 4 small points from source image
        pts_src = np.array([red_dot, yellow_dot, green_dot, blue_dot],dtype=float)

        #Coordinates in actual cartesian space where 1 is a scale term
        #pts_dst = np.array([[175, 50, 1],[275, -50, 1],[275, 50, 1],[175, -50, 1]]) #yaxis 50,-50
        pts_dst = np.array([[175, 50],[275, -50],[275, 50],[175, -50]], dtype=float) #yaxis 50,-50

        #Find location of objects to pick up...
        Order = ['red','yellow','green','blue']
        pts_target = np.array([red_target,yellow_target,green_target,blue_target])


        print 'pts_src = ', pts_src
        print 'pts_target = ', pts_target

        #Check if all points were seen
        n = len(pts_src)

        #assume we have all data
        do_homography = True

        for ii in range(n):
            if (pts_src[ii][0] == 0)&(pts_src[ii][1] == 0):
                #i.e. no sticker
                do_homography = False
                print('Not suitable for homography')

        if do_homography==True:
            print('Computing homography...')
            print 'pts_dst = ', pts_dst
            # Calculate Homography
            h, status = cv2.findHomography(pts_src, pts_dst) #h is the transform from that image to that
            

            print('Homography transform matrix is: ')
            print(h)

            # Warp source image to destination based on homography
            #im_homography = cv2.warpPerspective(imageBGR, h, (300,250))

            #Find location of objects to pick up...
            Order = ['red','yellow','green','blue']
            pts_target = np.array([red_target,yellow_target,green_target,blue_target])

            #pts_target = np.array([yellow_target,yellow_target,yellow_target,yellow_target])

            n = len(pts_target)

            
            print('pts_target = ')
            print(pts_target)

            if np.any(pts_target) is False:
                #i.e. no targets
                print('Table is clear! ')
            else:
                #Loop around the targets and compute real coordinate
                for ii in range(n):

                    if (pts_target[ii][0] == 0)&(pts_target[ii][1] == 0):
                        #if Output target is zero initialisation        
                        print('Did not see ' + Order[ii] + ' target')
                        
                    else:
                        #Do homography transform
                        Point = np.array([pts_target[ii][0],pts_target[ii][1],1])
                        
                        coordinate = np.dot(h,Point)

                        #Normalise
                        coordinate[0] = coordinate[0]/coordinate[2]
                        coordinate[1] = coordinate[1]/coordinate[2]
                        coordinate[2] = coordinate[2]/coordinate[2]                                   
                        
                        print(Order[ii] + ' target is at XYZ coordinate: ')
                        print(coordinate)
                        got_target = True
           


        #-----------------------------------------------------------------------------
        

        #Show frame
        #cv2.imshow('Colour Image',imageBGR)
        #cv2.imshow('Red Thresholding',red_mask)
        #cv2.imshow('Yellow Thresholding',yellow_mask) 
        #cv2.imshow('Green Thresholding',green_mask) 
        #cv2.imshow('Blue Thresholding',blue_mask) 
 
	# clear the stream in preparation for the next frame
        rawCapture.truncate(0)

print('Outputing value:')
print(coordinate)
 

