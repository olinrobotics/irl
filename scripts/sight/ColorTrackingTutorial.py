import cv2
import cv2.cv as cv
import numpy as np

def main():
	cap = cv2.VideoCapture(0)
	while True:
		ret, frame = cap.read()

		hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		blue_image = cv2.inRange(hsv_image, (0, 50, 50), (8, 170, 200))
		cv2.imshow("camera", frame)
		cv2.imshow("blue image", blue_image)
		c = cv2.waitKey(1)
		
if __name__ == '__main__':
	main()
