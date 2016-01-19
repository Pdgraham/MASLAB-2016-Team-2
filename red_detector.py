# import the necessary packages
from __future__ import print_function
import cv2

# load the image and convert it to grayscale
image = cv2.imread("blocks.jpg")
cv2.imshow("Original", image)
small = cv2.resize(image, (0,0), fx=0.5, fy=0.5) # half x and y axes
height, width, channels = small.shape
# BGR
for y in range(0,width):
	for x in range(0, height):
		red = small.item(x,y,2) 
		green = small.item(x,y,1)
		blue = small.item(x,y,0)
		if red > 1.3*green and red > 1.3*blue:
			small.itemset((x,y,2),0)
			small.itemset((x,y,1),0)
			small.itemset((x,y,0),0)

# draw the keypoints and show the output image
cv2.imshow("Output", image)
cv2.imshow("Small", small)
cv2.waitKey(0)