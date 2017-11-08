import cv2
import numpy as np
from Board import Board
import random

class Computer:

    def __init__(self, pn):
        self.player_num = pn
        self.must_sacrifice = False

    def play_move(self, board):
        possible_moves = self.non_sacrificial_moves(board)
        if len(possible_moves) == 0:
            self.must_sacrifice = True
        if self.must_sacrifice:
            return -1
        return possible_moves[random.randint(0,len(possible_moves))]


    def non_sacrificial_moves(self, board):
        possible_moves = []
        not_possible_moves = []
        for box in range(len(board.available_boxes)):
            if board.spaces_left(box) >= 3:
                for space in board.spaces_in_boxes[box]:
                    if board.available_spaces.get(space) and space not in not_possible_moves:
                        if space not in possible_moves:
                            possible_moves.append(space)
                    elif space not in not_possible_moves:
                        not_possible_moves.append(space)
            else:
                for space in board.spaces_in_boxes[box]:
                    if space not in not_possible_moves:
                        not_possible_moves.append(space)
        return possible_moves
