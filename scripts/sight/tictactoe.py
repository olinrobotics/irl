#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from edwin.msg import Edwin_Shape
import copy
import cv2
import cv2.cv as cv
import numpy as np
import random
import time

def inside_rect(rx, ry, w, h, x, y):
	# rx = rect[0]
	# ry = rect[1]
	if rx < x < rx+w and ry < y < ry+h:
		return True
	else:
		return False

class Game:
	def __init__(self):
		rospy.init_node('ttt_gamemaster', anonymous = True)
		self.draw_pub = rospy.Publisher('draw_cmd', Edwin_Shape, queue_size=10)
		self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)

		self.board_msg = Edwin_Shape()
		self.draw_msg = Edwin_Shape()
		self.draw_msg.shape = "square"

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

		self.image_rectangles = [(0, 0), (117, 0), (234, 0),
							(0, 96), (117, 96), (234, 96),
							(0, 192), (117, 192),(234, 192)]

		self.image_w = 117
		self.image_h = 96


	def draw_the_board(self):
		self.board_msg.shape = "board"
		self.board_msg.x = self.b_x
		self.board_msg.y = self.b_y
		#note that Z should be a function of y.
		self.board_msg.z = -630 - ((self.board_msg.y - 2500)/10)
		self.draw_pub.publish(self.board_msg)

		time.sleep(25)

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

	def edwin_move(self, index):
		#edwin moves to desired location and draws
		print "MOVING TO: ", index
		center = self.b_centers[index]
		self.draw_msg.x = center[0]
		self.draw_msg.y = center[1]
		#note that Z should be a function of y.
		self.draw_msg.z = -630 - ((self.draw_msg.y - 2500)/10)
		self.draw_pub.publish(self.draw_msg)

	 	self.board[index] = 10
	 	time.sleep(10)

	def manual_field_scan(self):
		#TODO: Update board with current values
		print self.board
		for i in range(9):
			if self.is_free(self.board, i):
				val_in = int(raw_input("Value at "+str(i)+": "))
				self.board[i] = val_in
				if val_in == 1:
					return


	def field_scan(self):
		time.sleep(5)
		motions = ["data: move_to:: 100, 2200, 500, 0",
					"data: rotate_wrist:: -100",
					"data: rotate_hand:: 50"]

		for motion in motions:
			print "pub: ", motion
			self.arm_pub.publish(motion)
			time.sleep(2)

		new_index = {}

		running = True
		while running:
			ret, frame = self.cap.read()

			imgray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
			ret,thresh = cv2.threshold(imgray,115,255,0)
			blur = cv2.blur(thresh, (2,2))

			contours,h = cv2.findContours(blur,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_TC89_L1)
			circles = cv2.HoughCircles(imgray, cv.CV_HOUGH_GRADIENT,1, 100, param1=50,param2=30,minRadius=20,maxRadius=50)

			if circles is not None:
				circles = np.round(circles[0, :]).astype("int")
				for (x, y, r) in circles:
					cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
					for i in range(9):
						if self.board[i] == 0:
							#checks each section of box if it contains a circle
							if inside_rect(self.image_rectangles[i][0], self.image_rectangles[i][1], self.image_w, self.image_h, int(x), int(y)):
								try:
									new_index[i] += 1
								except KeyError:
									new_index[i] = 1
								continue


			for rect in self.image_rectangles:
				cv2.rectangle(frame, rect, (rect[0]+self.image_w, rect[1]+self.image_h), (0,0,255), 2)

			cv2.imshow("camera", frame)
			c = cv2.waitKey(1)

			for key in new_index.keys():
				if new_index[key] == 5:
					self.board[key] = 1
					running = False
					#cv2.destroyAllWindows()


	def run(self):
		#Player is O: 1
		#Edwin is X: 10
		self.cap = cv2.VideoCapture(1)

	 	running = True
	 	turn = random.randint(0,1)
	 	self.draw_the_board()

	 	if turn == 0:
	 		#TODO: Add player turn notification
	 		print "Your turn first!"

	 	while running:
	 		if turn == 0: #Player turn.
	 			self.field_scan()
	 			turn = 1
	 			if self.is_winner(self.board) == 2: #Checks if the player made a winning move.
	 				#TODO: Add Edwin defeat notificaiton
	 				print "PLAYER WINS"
	 				running = False

	 		elif turn == 1: #Edwin's turn
	 			next_move_ind = self.next_move()
	 			self.edwin_move(next_move_ind)
	 			#TODO: Add player turn notification
	 			turn = 0
	 			if self.is_winner(self.board) == 1:
	 				#TODO: Add winning motion
	 				print "EDWIN WINS"
	 				running = False



if __name__ == '__main__':
	gm = Game()
	gm.run()