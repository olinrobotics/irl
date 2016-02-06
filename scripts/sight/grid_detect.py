#!/usr/bin/env python
import rospy
import cv2
import cv2.cv as cv
import numpy as np
import random
import time
import operator
import math
import itertools

from std_msgs.msg import String
from sensor_msgs.msg import Image
from edwin.msg import Edwin_Shape
from cv_bridge import CvBridge, CvBridgeError


def inside_rect(pt1, pt2, pt3, x, y):
	bax = pt2[0] - pt1[0]
	bay = pt2[1] - pt1[1]
	dax = pt3[0] - pt1[0]
	day = pt3[1] - pt1[1]

	if ((x - ax) * bax + (y - ay) * bay < 0.0):
		return False
	elif ((x - bx) * bax + (y - by) * bay > 0.0):
		return False
	elif ((x - ax) * dax + (y - ay) * day < 0.0):
		return False
	elif ((x - dx) * dax + (y - dy) * day > 0.0):
		return False

	return True


class GridDetector:
	def __init__(self, img):
		self.img = img
		self.boxes = []

	def get_distance(self, pt1, pt2):
		return math.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

	def get_pt_x(self, pt1, pt2, d):
		v = (pt1[0]-pt2[0], pt1[1]-pt2[1])
		v_mag = self.get_distance(pt1, pt2)

		u = (v[0]/v_mag, v[1]/v_mag)

		return (int(pt1[0]+d*u[0]), int(pt1[1]+d*u[1]))

	def get_box(self, ref_box, h, w):
		pt0 = ref_box[0]
		pt1 = ref_box[1]
		pt2 = self.get_pt_x(ref_box[1], ref_box[2], h)
		pt3 = self.get_pt_x(ref_box[0], ref_box[3], h)

		return [pt0, pt1, pt2, pt3]

	def get_grid(self, box):
		grid_height = int(self.get_distance(box[0], box[1]))
		grid_width = int(self.get_distance(box[1], box[3]))

		box2 = self.get_box([box[2], box[1], box[0], box[3]], grid_height, grid_width)
		box4 = self.get_box(box, grid_height, grid_width)
		box5 = self.get_box([box[3], box[2], box[1], box[0]], grid_height, grid_width)
		box7 = self.get_box([box[3], box[0], box[1], box[2]], grid_height, grid_width)

		box1 = self.get_box([box4[2], box4[1], box4[0], box4[3]], grid_height, grid_width)
		box6 = self.get_box([box4[3], box4[0], box4[1], box4[2]], grid_height, grid_width)

		box3 = self.get_box([box5[2], box5[1], box5[0], box5[3]], grid_height, grid_width)
		box8 = self.get_box([box5[3], box5[0], box5[1], box5[2]], grid_height, grid_width)

		return [box1, box2, box3, box4, box5, box6, box7, box8]

	def get_center_box(self):
		# img = cv2.imread(im_in)
		# img = im_in
		h, w, ch = self.img.shape

		gray = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)

		corners = cv2.goodFeaturesToTrack(gray,25,0.09,100)
		corners = np.int0(corners)

		dists = {}
		for num, i in enumerate(corners):
			x,y = i.ravel()
			# cv2.circle(img,(x,y),3,255,-1)
			dists[num] = math.sqrt((x-(w/2))**2 + (y-(h/2))**2)

		sorted_dists = sorted(dists.items(), key=operator.itemgetter(1))
		center_rect = []

		pts_distances = []

		for i in range(4):
			pts_distances.append(sorted_dists[i])
			center_rect.append(list(corners[pts_distances[i][0]].ravel()))

		# print center_rect

		rect = cv2.minAreaRect(np.int0(center_rect))

		box = cv2.cv.BoxPoints(rect)
		box = np.int0(box)

		boxes = self.get_grid(box)

		for box in boxes:
			box = np.int0(box)
			# cv2.drawContours(img,[box],0,(0,0,255),2)

		self.boxes = np.int0(boxes)
		return corners

	def run(self, img):
		self.img = img

class GridTester:
	def __init__(self):
		rospy.init_node('grid_locator', anonymous = True)
		self.draw_pub = rospy.Publisher('draw_cmd', Edwin_Shape, queue_size=10)
		self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)


		#Board X and Y positions
		self.b_x = 0
		self.b_y = 4000
		self.b_w = 250

		self.board_msg = Edwin_Shape()
		self.draw_msg = Edwin_Shape()


	def img_callback(self, data):
		try:
			self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)


	def field_scan(self):
		time.sleep(5)

		#pick marker off paper
		msg = "data: move_to:: 200, 2700, 1000, 0"
		# msg = "data: move_to:: " + str(self.b_x) + ", " + str(self.b_y) + ", " + str(self.board_msg.z+250) + ", " + str(0)
		print "sending: ", msg
		self.arm_pub.publish(msg)
		time.sleep(1)

		# motions = ["data: rotate_hand:: 50",
		# 			"data: rotate_wrist:: -550",
		# 			"data: move_to:: 200, "+self.y_pos+", 500, 0"]

		# for motion in motions:
		# 	print "pub: ", motion
		# 	self.arm_pub.publish(motion)
		# 	time.sleep(2)

		new_index = {}

		running = True
		while running:
			# ret, self.frame = self.cap.read()

			gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
			blur = cv2.GaussianBlur(gray, (5, 5), 0)

			height, width, channels = self.frame.shape
			self.image_w = width/3
			self.image_h = height/3

			contours,h = cv2.findContours(blur,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_TC89_L1)
			circles = cv2.HoughCircles(blur, cv.CV_HOUGH_GRADIENT,1, 100, param1=50,param2=30,minRadius=20,maxRadius=50)
			print len(circles)
			if circles is not None:
				circles = np.uint16(np.around(circles))

				circles = np.round(circles[0, :]).astype("int")
				for (x, y, r) in circles:
					cv2.circle(self.frame, (x, y), r, (0, 255, 0), 4)
					for i in range(9):
						if self.board[i] == 0:
							#checks each section of box if it contains a circle
							if inside_rect(self.image_rectangles[i][0], self.image_rectangles[i][1], self.image_rectangles[i][2], self.image_rectangles[i][3], int(x), int(y)):
								try:
									new_index[i] += 1
								except KeyError:
									new_index[i] = 1
								continue


			# for rect in self.image_rectangles:
				# cv2.rectangle(self.frame, rect, (rect[0]+self.image_w, rect[1]+self.image_h), (0,0,255), 2)

			# cv2.imshow("camera", self.frame)
			# c = cv2.waitKey(1)

			for key in new_index.keys():
				if new_index[key] == 5:
					self.board[key] = 1
					running = False

	def draw_the_board(self):
		# print "drawing the board"
		# self.board_msg.shape = "board"
		# self.board_msg.x = self.b_x
		# self.board_msg.y = self.b_y
		# #note that Z should be a function of y.
		# self.board_msg.z = -805 - ((self.board_msg.y - 2500)/10)
		# self.draw_pub.publish(self.board_msg)

		# time.sleep(25)

		# #look at grid
		## msg = "data: move_to:: 200, 2700, 1000, 0"
		# msg = "data: move_to:: 150, 2400, 1500, 0"
		# print "sending: ", msg
		# self.arm_pub.publish(msg)
		# time.sleep(1)

		gd = GridDetector(self.frame)
		corners = gd.get_center_box()
		self.image_rectangles = gd.boxes

		# print "CORNERS: ", corners
		# print "RECTANGLES: ", self.image_rectangles

		for i in range(20):
			gd.run(self.frame)
			corners = gd.get_center_box()
			for j in range(8):
				elem = gd.boxes[j]
				self.image_rectangles[j] = (self.image_rectangles[j] + elem)/2

			# self.image_rectangles = (np.array(self.image_rectangles) + np.array(gd.boxes))/2


		for box in self.image_rectangles:
			box = np.int0(box)
			cv2.drawContours(self.frame,[box],0,(0,0,255),2)

		for num, i in enumerate(corners):
			x,y = i.ravel()
			cv2.circle(self.frame,(x,y),3,255,-1)

		cv2.imshow("img", self.frame)
		c = cv2.waitKey(1)

		# print self.image_rectangles

	def run(self):
		time.sleep(2)
		# # r = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.draw_the_board()
		# 	self.field_scan()
		# 	# ret, frame = cap.read()
		# 	# img = self.get_center_box(frame)
			# cv2.imshow("img", self.frame)
			# c = cv2.waitKey(1)
		# 	time.sleep(1)
		# 	# r.sleep()

if __name__ == "__main__":
	gf = GridTester()
	gf.run()