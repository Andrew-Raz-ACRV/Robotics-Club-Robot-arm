import numpy as np
import cv2
from matplotlib import pyplot as plt

#--------------------------------------------------
#Get Grayscale image from Blue Green Red one
imageBGR = cv2.imread('DRC2016.jpg')
gray_image = cv2.cvtColor(imageBGR, cv2.COLOR_BGR2GRAY)
cv2.imshow('gray_image',gray_image) 
cv2.waitKey(0)                 # Waits forever for user to press any key
cv2.destroyAllWindows()        # Closes displayed windows
#-------------------------------------------------

#--------------------------------------------------
#Resizing Colour Images
imageBGR = cv2.imread('test.jpg') #be aware that cv2 reads colours in BGR

#Resize by 30%
scale = 0.3;
width = int(imageBGR.shape[1]*scale); #extract shape width and scale it
height = int(imageBGR.shape[0]*scale); #extract shape height and scale it
imageBGRs = cv2.resize(imageBGR, (width,height))

#Show image
cv2.imshow('imageBGRs',imageBGRs) 
cv2.waitKey(0)                 # Waits forever for user to press any key
cv2.destroyAllWindows()        # Closes displayed windows
#-------------------------------------------------

#-------------------------------------------------
#Thresholding Grayscale Images
img = cv2.imread('DRC2016.jpg',0) #0 makes it grayscale
ret,thresh1 = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
ret,thresh2 = cv2.threshold(img,127,255,cv2.THRESH_BINARY_INV)
ret,thresh3 = cv2.threshold(img,127,255,cv2.THRESH_TRUNC)
ret,thresh4 = cv2.threshold(img,127,255,cv2.THRESH_TOZERO)
ret,thresh5 = cv2.threshold(img,127,255,cv2.THRESH_TOZERO_INV)
titles = ['Original Image','BINARY','BINARY_INV','TRUNC','TOZERO','TOZERO_INV']
images = [img, thresh1, thresh2, thresh3, thresh4, thresh5]
for i in xrange(6):
    plt.subplot(2,3,i+1),plt.imshow(images[i],'gray')
    plt.title(titles[i])
    plt.xticks([]),plt.yticks([])
plt.show()
#-------------------------------------------------


#--------------------------------------------------
#The Colour Plane Thresholding

#We will threshold out The colour Blue

#Separate channels like this:
#b, g, r = imageBGR7[:, :, 0], imageBGR7[:, :, 1], imageBGR7[:, :, 2]

#Load test Image
imageBGR7 = cv2.imread('test7.jpg') #be aware that cv2 reads colours in BGR

#Threshold bounds note its (b,g,r) so 100<b<255, 0<g<80, 0<r<80
lower_bounds = np.array([100,0,0])
upper_bounds = np.array([255,80,80])
mask = cv2.inRange(imageBGR7,lower_bounds,upper_bounds)

#Resize images for display
scale = 0.3;
width = int(imageBGR7.shape[1]*scale); #extract shape width and scale it
height = int(imageBGR7.shape[0]*scale); #extract shape height and scale it
imageBGR7s = cv2.resize(imageBGR7, (width,height))
masks = cv2.resize(mask, (width,height)) #same size

#Show image
cv2.imshow('Colour Image',imageBGR7s) 
cv2.imshow('Thresholding in Colour',masks) 
cv2.waitKey(0)                 # Waits forever for user to press any key
cv2.destroyAllWindows()        # Closes displayed windows
#--------------------------------------------------


#--------------------------------------------------
#The HSV Thresholding and Bitwise operation

#We will threshold out The colour Blue

#Load test Image
imageBGR7 = cv2.imread('test7.jpg') #be aware that cv2 reads colours in BGR
#Convert to HSV
hsv = cv2.cvtColor(imageBGR7, cv2.COLOR_BGR2HSV)

#Threshold bounds note its (h,s,v) so 100<h<140, 50<s<255, 50<v<255
#Note Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255]
lower_bounds = np.array([100,50,50])
upper_bounds = np.array([140,255,255])
mask = cv2.inRange(hsv,lower_bounds,upper_bounds)

#Resize images for display
scale = 0.3;
width = int(imageBGR7.shape[1]*scale); #extract shape width and scale it
height = int(imageBGR7.shape[0]*scale); #extract shape height and scale it
imageBGR7s = cv2.resize(imageBGR7, (width,height))
masks = cv2.resize(mask, (width,height)) #same size

#Extra overlay binary image over colour image
# Bitwise-AND mask and original image
result = cv2.bitwise_and(imageBGR7s,imageBGR7s, mask= masks)

#Show image
cv2.imshow('Colour Image',imageBGR7s) 
cv2.imshow('HSV Thresholding',result) 
cv2.waitKey(0)                 # Waits forever for user to press any key
cv2.destroyAllWindows()        # Closes displayed windows
#--------------------------------------------------

##--------------------------------------------------
##The Colour Plane Chromaticity
#
##Load test Image
#imageBGR7 = cv2.imread('test7.jpg') #be aware that cv2 reads colours in BGR
#
##Separate channels like this:
#B, G, R = imageBGR7[:, :, 0], imageBGR7[:, :, 1], imageBGR7[:, :, 2]
#
##Find size of image
#width = imageBGR7.shape[1]
#height = imageBGR7.shape[0]
#
##Initialise
#r = np.zeros((height, width))
#g = np.zeros((height, width))
#b = np.zeros((height, width))
#
##Loop around the image pixels
#for j in range(0,width,1):
#    for i in range(0,height,1):
#        #Chromaticity functions
#        r[i,j] = int(255* R[i,j]/(R[i,j]+G[i,j]+B[i,j]))
#        g[i,j] = int(255* G[i,j]/(R[i,j]+G[i,j]+B[i,j]))
#        b[i,j] = int(255* B[i,j]/(R[i,j]+G[i,j]+B[i,j]))
#
##Merge the Image as one again
#Chroma = np.zeros((height, width, 3))
#Chroma[:, :, 0] = b
#Chroma[:, :, 1] = g
#Chroma[:, :, 2] = r
#
#
##Resize images for display
#scale = 0.3;
#width = int(imageBGR7.shape[1]*scale); #extract shape width and scale it
#height = int(imageBGR7.shape[0]*scale); #extract shape height and scale it
#imageBGR7s = cv2.resize(imageBGR7, (width,height))
#Chromas = cv2.resize(Chroma, (width,height)) #same size
#
##Show image
#cv2.imshow('Colour Image',imageBGR7s) 
#cv2.imshow('Chromaticity',Chromas) 
#cv2.waitKey(0)                 # Waits forever for user to press any key
#cv2.destroyAllWindows()        # Closes displayed windows
##--------------------------------------------------
#
