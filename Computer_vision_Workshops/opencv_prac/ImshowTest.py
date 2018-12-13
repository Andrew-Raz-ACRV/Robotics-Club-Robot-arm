import numpy as np
import cv2
from matplotlib import pyplot as plt

#be aware that cv2 reads colours in BGR so convert to RGB
imageBGR = cv2.imread('test.jpg')

#Resize by 40%
width = int(imageBGR.shape[1]*0.3);
height = int(imageBGR.shape[0]*0.3);
imageBGRs = cv2.resize(imageBGR, (width,height))

#Show image
cv2.imshow('imageBGRs',imageBGRs) 
cv2.waitKey(0)                 # Waits forever for user to press any key
cv2.destroyAllWindows() 



imageBGR = cv2.imread('DRC2016.jpg')
gray_image = cv2.cvtColor(imageBGR, cv2.COLOR_BGR2GRAY)
cv2.imshow('gray_image',gray_image) 
cv2.waitKey(0)                 # Waits forever for user to press any key
cv2.destroyAllWindows()        # Closes displayed windows

# =============================================================================
# b, g, r    = image[:, :, 0], image[:, :, 1], image[:, :, 2] # For RGB image
# imageRGB = cv2.cvtColor(imageBGR, cv2.COLOR_BGR2RGB)
#plt.imshow(imageRGB)
#plt.show()

# rgbArray = np.zeros(image.shape, 'uint8')
# rgbArray[..., 0] = r
# rgbArray[..., 1] = g
# rgbArray[..., 2] = b
# 
# img = rgbArray
# =============================================================================


#plt.hist(imageRGB.ravel(),256,[0,256]); 
#plt.show()
#plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
#