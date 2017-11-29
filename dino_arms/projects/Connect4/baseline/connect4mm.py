#!/usr/bin/env python

# import rospy
# import rospkg
import numpy as np
import pandas as pd
import random
import time
# from std_msgs.msg import String, Int16
import cPickle as pickle
import argparse
from game_board import C4Board
from Queue import *
from minimax import Minimax


class Connect4(object):

    def __init__(self, render=True):
        self.board = C4Board(render)
        self.max = Minimax()

    def run(self):

        print " I WILL GO FIRST"
        move, value = self.max.bestMove(5, self.board.reset())
        observation_, done = self.board.step(move, 1)

        while True:
            player_action = None
            while player_action not in range(1,8):
                player_action = int(raw_input("YOUR TURN, WHAT'S YOUR MOVE? (1-7) "))
                print "INVALID MOVE" if player_action not in range(1,8) else ""

            observation_, done = self.board.step(player_action-1, 2)

            # break if I win
            if done:
                print "\n"*2
                print "HUMAN WINS"
                break

            move, value = self.max.bestMove(5, observation_)
            observation_, done = self.board.step(move, 1)

            # break if ai wins
            if done:
                print "\n"*2
                print "AI WINS"
                break

        print "GAME OVER"


if __name__ == "__main__":
    connect = Connect4()
    connect.run()
