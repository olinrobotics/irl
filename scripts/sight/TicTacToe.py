#!/usr/bin/env python
import rospy
from std_msgs import String
import copy
import cv2
import cv2.cv as cv
import numpy as np
import random
import time

class Game:
	def __init__(self):
        rospy.init_node('ttt_gamemaster', anonymous = True)
        self.pub = rospy.Publisher('draw_cmd', String, queue_size=10)

        #board represented by 9 element list
        #A blank space is represented by 0, an "O" is 1, and "X" is 10, we start with blank board
		self.board =   [0, 0, 0,
						0, 0, 0,
						0, 0, 0]

		self.corners = [0, 2, 6, 8] #indices of the corner locations
		self.sides = [1,3,5,7]
		self.middle = [4]

	def is_winner(board):
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
		board_sums[6] = board[0] + board[4] + board[7] #LR Diagonal Line
		board_sums[7] = board[2] + board[4] + board[6] #RL Diagonal Line
		#There are 4 cases - NoWin, EdWin, Player Win and Tie, Tie is accounted for
		#in a piece of code further down.
		for x in range(0, len(board_sums)):
			if (board_sums(x) == 30):
				return "Ed wins!"
			elif (board_sums(x) == 3):
				return "You win!"
			else:
				return False

	def is_free(self, board, index):
	 	#Function is to check whether a space is occupied or not.
	 	if board[index] == 0
	 		return True
	 	else
	 		return False

	def is_board_full(self):
	 	for i in range(0, 8):
	 		if self.is_free(self.board, i):
	 			return False
	 	return True


	def next_move(self):
		#Returns index of next move

		#Checks if Edwin can win on this move
		for i in range(0, len(self.board)):
			board_copy = copy.deepcopy(self.board)
			if is_free(board_copy, i):
				board_copy[i] = 10
				if self.is_winner(board_copy) == 1:
					return i

		#Checks if player can win the next turn
		for i in range(0, len(self.board)):
			board_copy = copy.deepcopy(self.board)
			if is_free(board_copy, i):
				board_copy[i] = 1:
				if self.is_winner(board_copy) == 2:
					return i

		#Otherwise, prioritizes grabbing corners.
		for i in range(0, len(self.corners)):
			board_copy = copy.deepcopy(self.board)
			if is_free(board_copy, self.corners[i]):
				return i

		#Otherwise, get the middle.
		board_copy = copy.deepcopy(self.board)
		if is_free(board_copy, self.middle):
			return self.middle

		#Otherwise, get a side.
		for i in range(0, len(self.sides)):
			board_copy = copy.deepcopy(self.board)
			if is_free(board_copy, self.sides[i]):
				return i


	def edwin_move(self, index):
		#edwin moves to desired location and draws
	 	self.board[index] = 10

	def run(self):
	 	running = True
	 	turn = random.randint(0,1)
	 	if turn == 0:
	 		#TODO: Add player turn notification
	 		pass

	 	while running:
	 		if turn == 0: #Player turn.
	 			self.field_scan()
	 			turn = 1
	 			if is_winner(self.board): #Checks if the player made a winning move.
	 				#TODO: Add Edwin defeat notificaiton
	 				running = False
	 		elif turn == 1: #Edwin's turn
	 			next_move = self.next_move()
	 			self.edwin_move(next_move)
	 			#TODO: Add player turn notification
	 			turn = 0
	 			if is_winner(self.board):
	 				#TODO: Add winning motion
	 				running = False


	def field_scan(self):
		#TODO: Update board with current values
		self.board = 1
		pass

