import cv2
import numpy as np

cap = cv2.VideoCapture(1)

while True:
	ret, im = cap.read()
	gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)

	imgSplit = cv2.split(im)
	flag,b = cv2.threshold(imgSplit[2],0,255,cv2.THRESH_OTSU)

	element = cv2.getStructuringElement(cv2.MORPH_CROSS,(1,1))
	cv2.erode(b,element)

	edges = cv2.Canny(b,150,200,3,5)
	img = im.copy()

	lines = cv2.HoughLinesP(edges,1,np.pi/2,2, minLineLength = 200, maxLineGap = 100)[0]

	# for x1,y1,x2,y2 in lines:
	# 	for index, (x3,y3,x4,y4) in enumerate(lines):

	# 		if y1==y2 and y3==y4: # Horizontal Lines
	# 			diff = abs(y1-y3)
	# 		elif x1==x2 and x3==x4: # Vertical Lines
	# 			diff = abs(x1-x3)
	# 		else:
	# 			diff = 0

	# 		if diff < 10 and diff is not 0:
	# 			del lines[index]

	# gridsize = (len(lines) - 2) / 2

	for x1,y1,x2,y2 in lines:
		cv2.line(img,(x1,y1),(x2,y2),(0,255,0),1)

	cv2.imshow('houghlines',img)
	c = cv2.waitKey(1)

cv2.destroyAllWindows()