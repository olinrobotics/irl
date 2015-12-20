import copy
import cv2
import cv2.cv as cv
import numpy as np
import random
import time

class Game:
	 def __init__(self):
	 self.board = 	[0, 0, 0,
	 				 0, 0, 0,
	 				 0, 0, 0]

	 #The field is represented by an integer array 0-8.
	 #Edwin will be using X.
	 #A blank space is represented by 0, a "O" is 1, and "X" is 10.
	 self.corners = [0, 2, 6, 8] #indices of the corner locations
	 self.sides = [1,3,5,7]
	 self.middle = [4]

	 def is_winner(board):
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
	 		if (board_sums(x) == 30) or (board_sums(x) == 3):
	 			return True
	 	return False

	 def bot_move(self):
	 	#Like in the example, the priority is for the bot to win, then neutralize.
	 	#This function will return the index for the bot.

	 	#Checks if the bot can win on this move
	 	for i in range(0, len(self.board)):
	 	board_copy = copy.deepcopy(self.board)
	 		if is_free(board_copy, i):
	 		board_copy[i] = 10
	 			if self.is_winner(board_copy) == True:
	 				return i

		 #Checks if the winner can win the next turn
		 for i in range(0, len(self.board)):
		 	board_copy = copy.deepcopy(self.board)
		 	if is_free(board_copy, i):
		 		board_copy[i] = 1:
		 		if self.is_winner(board_copy) == True:
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

	 def actually_move(self, board, index): #Edwin's move.
	 	board[index] = 10


	 def start_game_loop(self):
	 	Game_not_over = True
	 	#Edwin will make a nudging gesture every time it's the player's turn.
	 	turn = random.randint(0,1)
	 	if turn == 0:
	 		Edwin.Nudge #Placeholder to tell that user is going first.

	 	while Game_not_over
	 		if turn == 0: #Player turn.
	 			while self.FieldScan() == self.board:
	 				time.sleep(1)
	 			self.FieldScan() = self.board #updates the board.
	 			turn = 1
	 		#Basically detect when a player made change has occured on the board.
	 		#Interaction happens during the Fieldscanning.


	 		#Games end on Edwin's turn, wherein he either ends the game or realizes
	 		#that the game is over.
	 		if turn == 1:
	 			if is_winner(self.board): #Checks if the player made a winning move.
	 				Edwin.defeat()
	 				Game_not_over = False
	 				break

	 			Edwin.move(self.bot_move) #Placeholder for Edwin to move/draw his move.
	 			self.actually_move(self.board, self.bot_move)
	 			if is_winner(self.board):
	 				Edwin.taunt()
	 				Game_not_over = False
	 				break
	 			else:
	 				Edwin.Nudge()
	 				turn = 0


# #------------------------------------------------------------------------------------
# #RealWorldInteraction Section

	def FieldScan():
		# FieldScan returns the digitized 0-8 array as a board.
		#I suppose FieldScan works on the assumption that the human doesn't cheat.
		#Technically, Edwin will look for the grid, assign each box a number in the
		#array, and then
		cap = cv2.VideoCapture(0)
		while True:
			ret, frame = cap.read()

			retGray, gray = cv2.threshold(frame, 127, 255, 1)
			contours, h = cv.findContours(gray, 1, 2)

			for cont in contours:
				approx = cv2.approxPolyDP(cont, 0.01*cv2.arcLength(cnt, True), True)
				sides = len(approx)
				if sides == 4:
					cv2.drawContours(frame, [cont], 0, 255, -1)
				elif sides > 15:
					cv2.drawContours(frame, [cont], 0, (0, 255, 255), -1)

			cv2.imshow("camera", frame)
			cv2.imshow("gray", gray)
			c = cv2.waitKey(1)




