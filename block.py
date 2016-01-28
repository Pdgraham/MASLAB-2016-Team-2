class Block:
	def __init__(self, x, y):
		self.numPixelsInBlock = 0
		self.topLeftCornerY = 1200
		self.topLeftCornerX = 1200
		self.bottomRightCornerY = 0
		self.bottomRightCornerX = 0
		self.accumulatedX = 0
		self.accumulatedY = 0
                self.meanX = 0
                self.minY = 0
		self.color = ""
		self.addPixelToBlock(x,y)

        def getBlockBottomPixel(self):
                return self.meanX, self.minY

	def getBlockAttributes(self):
		return self.color, self.numPixelsInBlock, self.topLeftCornerY, self.topLeftCornerX, self.bottomRightCornerY, self.accumulatedX, self.accumulatedY

	def getNumPixelsInBlock(self):
		return self.numPixelsInBlock

	def getColor(self):
		return self.color

	def setColor(self, color):
		self.color = color

	def addPixelToBlock(self, x, y):
		self.accumulatedX += x
		self.accumulatedY += y
		if x + y < self.topLeftCornerX + self.topLeftCornerY:
			self.topLeftCornerX = x
			self.topLeftCornerY = y
		if x + y > self.bottomRightCornerX + self.bottomRightCornerY:
			self.bottomRightCornerX = x
			self.bottomRightCornerY = y
                if y > self.minY:
                        self.minY = y
		self.numPixelsInBlock += 1
