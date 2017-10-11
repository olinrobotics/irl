#!/usr/bin/env python


import rospy
import numpy as np
import random
import time
from std_msgs.msg import String, Int16

class C4Board(object):

    def __init__(self):
        self.action_space = [0,1,2,3,4,5,6]
        self.n_actions = len(self.action_space)
        self.init_board()


    def init_board(self):
        self.board = []
        for i in range(7):
            column = [0]*6
            self.board.append(column)
        self.column_stack = [5,5,5,5,5,5,5]


    def reset(self):
        self.render()
        self.init_board


    def make_move(self, column, player):
        move = (column, self.column_stack[column])
        self.board[column][self.column_stack[column]] = player
        self.column_stack[column] -= 1
        return self.board, move


    def connect_4(self, move):
        # TODO: Write this function


    def step(self, action, player):
        s_, move = self.make_move(action, player)

        if self.connect_4(move) and player == 1:
            reward1 = 1
            reward2 = -1
            done = True
        elif self.connect_4(move) and player == 2:
            reward1 = -1
            reward2 = 1
            done = True
        else:
            reward1 = reward2 = 0
            done = False

        return s_, reward1, reward2, done


    def render(self):
        time.sleep(0.1)
        for i in range(6):
            for j in range(7):
                piece = self.board[j][i]
                if piece == 0:
                    print "0"
                else:
                    print "R" if piece == 1 else "B"
                print " "

            print "\n"
