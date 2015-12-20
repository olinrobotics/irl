import cv2
import cv2.cv as cv
import numpy as np


def FieldScan():
	# FieldScan returns the digitized 0-8 array as a board.
	#I suppose FieldScan works on the assumption that the human doesn't cheat.
	#Technically, Edwin will look for the grid, assign each box a number in the
	#array, and then
	cap = cv2.VideoCapture(0)
	while True:

		ret, frame = cap.read()

		imgray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		#thresh = imgray
		ret,thresh = cv2.threshold(imgray,127,255,0)
		contours,h = cv2.findContours(thresh,1,2)

		#contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		# gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		# thresh = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
		# im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		#contours, h = cv2.findContours(thresh, 2, 3)

		for cont in contours:
			approx = cv2.approxPolyDP(cont, 0.01*cv2.arcLength(cont, True), True)
			sides = len(approx)
			print sides
			if sides > 6:
				print "drawing circle"
				cv2.drawContours(frame, [cont], 0, (0, 255, 0), -1)
			elif sides == 4:
				cv2.drawContours(frame, [cont], 0, (0, 255, 255), -1)


		cv2.imshow("camera", frame)
		c = cv2.waitKey(1)

if __name__ == '__main__':
	FieldScan()
