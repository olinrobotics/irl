#!/usr/bin/env python
import rospy
import copy
import cv2
import cv2.cv as cv
import numpy as np
import random
import time
import math
import operator
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

		corners = cv2.goodFeaturesToTrack(gray,25,0.01,120)
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

class Game:
	def __init__(self):
		rospy.init_node('ttt_gamemaster', anonymous = True)
		self.draw_pub = rospy.Publisher('draw_cmd', Edwin_Shape, queue_size=10)
		self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
		self.behav_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)

		self.board_msg = Edwin_Shape()
		self.draw_msg = Edwin_Shape()

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)

		#Board X and Y positions
		self.b_x = 0
		self.b_y = 4000
		self.b_w = 250

		#Sector centroids
		self.b_centers = {}
		self.b_centers[0] = (self.b_x - 2.25*self.b_w, self.b_y + 1.5*self.b_w)
		self.b_centers[1] = (self.b_x, self.b_y + 2*self.b_w)
		self.b_centers[2] = (self.b_x + 2*self.b_w, self.b_y + 1.5*self.b_w)

		self.b_centers[3] = (self.b_x - 2*self.b_w, self.b_y)
		self.b_centers[4] = (self.b_x, self.b_y)
		self.b_centers[5] = (self.b_x + 2*self.b_w, self.b_y)

		self.b_centers[6] = (self.b_x - 2*self.b_w, self.b_y - 1.5*self.b_w)
		self.b_centers[7] = (self.b_x, self.b_y - 1.5*self.b_w)
		self.b_centers[8] = (self.b_x + 2*self.b_w, self.b_y - 1.5*self.b_w)


		#A blank space is represented by 0, an "O" is 1, and "X" is 10, we start with blank board
		self.board =   [0, 0, 0,
						0, 0, 0,
						0, 0, 0]

		self.corners = [0,2,6,8] #indices of the corner locations
		self.sides = [1,3,5,7]
		self.middle = 4

		self.y_pos = 2500
		self.ok = "n"

	def img_callback(self, data):
		try:
			self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)


	def is_winner(self, board):
		"""
		Computes whether board will win
		"""

		board_sums = [0, 0, 0, 0, 0, 0, 0, 0] #These are the 8 different lines.
		board_sums[0] = board[0] + board[1] + board[2] #Top Horizontal Line
		board_sums[1] = board[3] + board[4] + board[5] #Middle Horizontal Line
		board_sums[2] = board[6] + board[7] + board[8] #Bottom Horizontal Line
		board_sums[3] = board[0] + board[3] + board[6] #Left Vertical Line
		board_sums[4] = board[1] + board[4] + board[7] #Middle Vertical Line
		board_sums[5] = board[2] + board[5] + board[8] #Right Vertical Line
		board_sums[6] = board[0] + board[4] + board[8] #LR Diagonal Line
		board_sums[7] = board[2] + board[4] + board[6] #RL Diagonal Line
		#There are 4 cases - NoWin, EdWin, Player Win and Tie, Tie is accounted for
		#in a piece of code further down.

		for b_sum in board_sums:
			if b_sum == 30:
				return 1
			elif b_sum == 3:
				return 2
		return False

	def is_free(self, board, index):
	 	#Function is to check whether a space is occupied or not.
	 	if board[index] == 0:
	 		return True
	 	else:
	 		return False

	def is_board_full(self):
	 	for i in range(0, 8):
	 		if self.is_free(self.board, i):
	 			return False
	 	return True

	def next_move(self):
		#Returns index of next move
		#Checks if Edwin can win on this move
		for i in range(len(self.board)):
			board_copy = copy.deepcopy(self.board)
			if self.is_free(board_copy, i):
				board_copy[i] = 10
				if self.is_winner(board_copy) == 1:
					return i

		#Checks if player can win the next turn
		for i in range(len(self.board)):
			board_copy = copy.deepcopy(self.board)
			if self.is_free(board_copy, i):
				board_copy[i] = 1
				if self.is_winner(board_copy) == 2:
					return i

		#Otherwise, prioritizes grabbing corners.
		for i in range(len(self.corners)):
			board_copy = copy.deepcopy(self.board)
			if self.is_free(board_copy, self.corners[i]):
				return self.corners[i]

		#Otherwise, get the middle.
		board_copy = copy.deepcopy(self.board)
		if self.is_free(board_copy, self.middle):
			return self.middle

		#Otherwise, get a side.
		for i in range(0, len(self.sides)):
			board_copy = copy.deepcopy(self.board)
			if self.is_free(board_copy, self.sides[i]):
				return self.sides[i]



	def manual_field_scan(self):
		#TODO: Update board with current values
		print self.board
		for i in range(9):
			if self.is_free(self.board, i):
				val_in = int(raw_input("Value at "+str(i)+": "))
				self.board[i] = val_in
				if val_in == 1:
					return

	def calibrate(self):
		time.sleep(5)
		running = True

		while "y" not in self.ok:
			self.y_pos = str(raw_input("Y position: "))
			motions = ["data: rotate_hand:: 50",
				"data: rotate_wrist:: -500",
				"data: move_to:: 200, "+self.y_pos+", 500, 0"]

			for motion in motions:
				print "pub: ", motion
				self.arm_pub.publish(motion)
				time.sleep(2)

			# ret, self.frame = self.cap.read()
			for rect in self.image_rectangles:
				cv2.rectangle(self.frame, rect, (rect[0]+self.image_w, rect[1]+self.image_h), (0,0,255), 2)

			cv2.imshow("camera", self.frame)
			c = cv2.waitKey(1)
			self.ok = str(raw_input("OK?"))


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
		print "drawing the board"
		self.board_msg.shape = "board"
		self.board_msg.x = self.b_x
		self.board_msg.y = self.b_y
		#note that Z should be a function of y.
		self.board_msg.z = -785 - ((self.board_msg.y - 2500)/10)
		self.draw_pub.publish(self.board_msg)

		time.sleep(25)

		#look at grid
		msg = "data: move_to:: 200, 2700, 1000, 0"
		print "sending: ", msg
		self.arm_pub.publish(msg)
		time.sleep(1)

		gd = GridDetector(self.frame)
		gd.get_center_box()
		self.image_rectangles = gd.boxes

	def edwin_move(self, index):
		#edwin moves to desired location and draws
		print "MOVING TO: ", index
		center = self.b_centers[index]
		self.draw_msg.shape = "square"
		self.draw_msg.x = center[0]
		self.draw_msg.y = center[1]
		#note that Z should be a function of y.
		self.draw_msg.z = -794 - ((self.draw_msg.y - 2500)/10)
		self.draw_pub.publish(self.draw_msg)

	 	self.board[index] = 10
	 	time.sleep(10)

	def ai_move(self, index):
		#edwin moves to desired location and draws
		print "MOVING TO: ", index
		center = self.b_centers[index]
		self.draw_msg.shape = "circle"
		self.draw_msg.x = center[0]
		self.draw_msg.y = center[1]
		#note that Z should be a function of y.
		self.draw_msg.z = -794 - ((self.draw_msg.y - 2500)/10)
		self.draw_pub.publish(self.draw_msg)

	 	self.board[index] = 1
	 	time.sleep(10)


	def run(self):
		#Player is O: 1
		#Edwin is X: 10
		# self.cap = cv2.VideoCapture(1)
	 	running = True
	 	turn = random.randint(0,1)
	 	time.sleep(5)
	 	self.draw_the_board()
	 	# self.calibrate()
	 	self.y_pos = "2400"
	 	print "running"

	 	if turn == 0:
	 		print "Your turn first!"

	 	ai = False
	 	while running:
	 		self.field_scan()
	 		# print self.board
	 		if turn == 0: #Player turn.
	 			#self.behav_pub.publish("nudge")
	 			#time.sleep(10)
	 			if ai:
	 				ai_next_move_ind = self.next_move()
	 				self.ai_move(ai_next_move_ind)
	 			else:
	 				self.field_scan()
	 			if self.is_winner(self.board) == 2: #Checks if the player made a winning move.
	 				print "PLAYER WINS"
	 				self.behav_pub("sad")
	 				running = False
	 			turn = 1
	 			continue

	 		elif turn == 1: #Edwin's turn
	 			next_move_ind = self.next_move()
	 			self.edwin_move(next_move_ind)
	 			turn = 0
	 			if self.is_winner(self.board) == 1:
	 				print "EDWIN WINS"
	 				self.behav_pub("gloat")
	 				running = False



if __name__ == '__main__':
	gm = Game()
	gm.run()