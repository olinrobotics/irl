import cv2
import cv2.cv as cv
import numpy as np
import copy


def FieldScan():
	# FieldScan returns the digitized 0-8 array as a board.
	#I suppose FieldScan works on the assumption that the human doesn't cheat.
	#Technically, Edwin will look for the grid, and assign each box a number in the
	#array.
	cap = cv2.VideoCapture(0)
	while True:
		#This program should find the large tic tac toe box,break it down into
		#9 different cells, and then analyze the contents of each cell.  
		ret, frame = cap.read()

		imgray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		ret,thresh = cv2.threshold(imgray,115,255,0)
		blur = cv2.blur(thresh, (2,2))

		contours,h = cv2.findContours(blur,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_TC89_L1)

		for cont in contours:
			approx = cv2.approxPolyDP(cont, 0.01*cv2.arcLength(cont, True), True)
			sides = len(approx)
			if (sides > 15):
				c, r = cv2.minEnclosingCircle(cont)
				c2 = np.round(c).astype("int")
				radius = np.round(r).astype("int")
				area = cv2.contourArea(cont)
				filterLevel = int(area - 3.14*radius**2)
				if filterLevel < 2:
					cv2.circle(frame, (c2[0], c2[1]), radius, (0, 255, 0), 4)
			elif sides == 4:
				cv2.drawContours(frame, [cont], 0, (0, 255, 255), -1)

		#Still needs a return function.  


		cv2.imshow("camera", frame)
		cv2.imshow("thresh", thresh)
		cv2.imshow("contour", blur)
		c = cv2.waitKey(1)

if __name__ == '__main__':
	FieldScan()
