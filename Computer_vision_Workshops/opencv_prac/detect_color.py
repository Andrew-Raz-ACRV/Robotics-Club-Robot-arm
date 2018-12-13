# USAGE
# python detect_color.py --image pokemon_games.png

# import the necessary packages
import numpy as np
import cv2
from matplotlib import pyplot as plt

# load the image
image = cv2.imread('test1.jpg')



plt.imshow(image)
plt.show()

# define the list of boundaries
boundaries = [
	([17, 15, 100], [50, 56, 200]),
	([86, 31, 4], [220, 88, 50]),
	([25, 146, 190], [62, 174, 250]),
	([103, 86, 65], [145, 133, 128])
]

# loop over the boundaries
for (lower, upper) in boundaries:
	# create NumPy arrays from the boundaries
	lower = np.array(lower, dtype = "uint8")
	upper = np.array(upper, dtype = "uint8")

	# find the colors within the specified boundaries and apply
	# the mask
	mask = cv2.inRange(image, lower, upper)
	output = cv2.bitwise_and(image, image, mask = mask)

	# show the images
	plt.imshow("images", np.hstack([image, output]))
	cv2.waitKey(0)