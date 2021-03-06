# import the necessary packages
from __future__ import print_function
import numpy
import cv2
import time
from block import Block 
import sys

def FloodFill(x,y,image,block):
	height, width, cn = image.shape;
	# print("width", width)
	# print("height", height)
	# print("x: ", x)
	# print("y: ", y)
	# bgrPixel = image[y,x] # B
	# print(bgrPixel)
	red = image.item(y,x,2) 
	green = image.item(y,x,1)
	blue = image.item(y,x,0)
	pixelIsRed = red > 1.3*green and red > 1.3*blue
	# pixelIsGreen = green > 1.5*red and green > 1.5*blue
	pixelIsGreen = False

	blockColor = block.getColor()
	if block.getColor() == "":
		# print("setting new block color")
		if pixelIsRed:
			blockColor = "red"
			block.setColor(blockColor)
		elif pixelIsGreen:
			blockColor = "green"
			block.setColor(blockColor)
	
	if pixelIsGreen or pixelIsRed:
		if pixelIsRed:
			# print ("Setting pixel black")
			newColor = 0
		if pixelIsGreen:
			newColor = 255
		image.itemset((y,x,2),newColor)
		image.itemset((y,x,1),newColor)
		image.itemset((y,x,0),newColor)

		block.addPixelToBlock(x,y)
		# print("! numPixels: ", block.getNumPixelsInBlock())

		for dx in xrange(-1,2,1):
			for dy in xrange(-1,2,1):
				# print("dx: ", dx)
				# print("dy: ", dy)
				if (dx == 0 and dy == 0):
					continue
				if (x+dx<width and x+dx>=0) and (y+dy<height and y+dy>=0):
					if (image.item(y+dy,x+dx,0) != 0 and image.item(y+dy,x+dx,0) != 255):
						FloodFill(x+dx,y+dy,image,block)

	return

def CalculateBlocks(video_image):
	start = time.time()
        orig_image = video_image
	# orig_image = cv2.imread("block_test.jpg")
	# orig_image = cv2.imread("Picture 11.jpg") # Picture 6-11
	# cv2.imshow("Original", orig_image)
	image = cv2.resize(orig_image, (0,0), fx=0.25, fy=0.25) # half x and y axes
	height, width, channels = image.shape
	# BGR
	# for y in range(0, width):
	# 	for x in range(0, height):
	# 		red = image.item(x,y,2) 
	# 		green = image.item(x,y,1)
	# 		blue = image.item(x,y,0)
	# 		if red > 1.3 * green and red > 1.3 * blue:
	# 			image.itemset((x,y,2),0)
	# 			image.itemset((x,y,1),0)
	# 			image.itemset((x,y,0),0)

	blocks = []
	for x in xrange(10,width-10,10):
		for y in xrange(10,height-10,10):
			block = Block(x,y);

			# print("x,y:", x,",",y)
			FloodFill(x,y,image,block)

			# print("numPixels: ", block.getNumPixelsInBlock())
			if block.getNumPixelsInBlock() > 500:
                                block.meanX = block.accumulatedX / block.getNumPixelsInBlock()
				blocks.append(block)
                                # x, y = block.getBlockBottomPixel()
				# print(x,y)
                                # print (block.meanX == x)
                                # image.itemset((y, x, 0),0)
                                # image.itemset((y, x, 1),255)
                                # image.itemset((y, x, 2),0)

	# print("Blocks: ", len(blocks))
	end = time.time()
	# print("Time: ", (end - start))
	return blocks

def main():
	sys.setrecursionlimit(10000) # 10000 is an example, try with different values; default is 1000
        cam = cv2.VideoCapture(0)
        ret, frame = cam.read()
#        img = cv2.resize(frame,None,fx=0.25, fy=0.25, interpolation = cv2.INTER_AREA)
        cv2.imshow("image", frame)
        cv2.waitKey(0)
        print("VideoFrame captured: ", ret)
	blocks = CalculateBlocks(ret)
        for block in blocks:
        	x, y =  block.getBlockBottomPixel()
		print((x,y))

if __name__ == "__main__":
    main()
