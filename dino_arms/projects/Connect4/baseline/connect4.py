#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import pandas as pd
import random
import time
from std_msgs.msg import String, Int16
import cPickle as pickle
import argparse
from game_board import C4Board
from Queue import *
from minimax import Minimax

"""
The Connect 4 Game module
by Kevin Zhang and Hannah Kolano

to run you must connect to Draco, the dragon. see github for how to do that

also requires the following to be run:
1. roscore
2. rosrun rosserial_python serial_node.py /dev/ttyACM0
3. rosrun irl ur5_arm_node.py

then run this script

basic steps:
1. plays connect 4, will always go second
2. waits for an input for a human, then calculates and executes its own move
3. repeats until a winner is found
"""


class Connect4(object):
    """
    the Connect 4 game, with computer vision for understanding the game board and
    whose turn it is, minimax for planning its own moves, and basic control system
    to make those moves and other interactions
    """

    def __init__(self, render=True):
        self.board = C4Board(render)
        self.max = Minimax()


    def play_game(self):
        """
        plays a single game of connect 4
        """

        #TODO make the move poses for Draco, whether it be XYZ or joints, there will be 7

        self.board.reset()

        while True:
            player_action = None
            while player_action not in range(1,8):
                player_action = int(raw_input("YOUR TURN, WHAT'S YOUR MOVE? (1-7) "))
                print "INVALID MOVE" if player_action not in range(1,8) else ""

            observation_, done = self.board.step(player_action-1, 1)

            # break if I win
            if done:
                print "\n"*2
                print "HUMAN WINS"
                break

            move, value = self.max.bestMove(5, observation_)
            observation_, done = self.board.step(move, 2)

            # break if ai wins
            if done:
                print "\n"*2
                print "AI WINS"
                break

        print "GAME OVER"


    def run(self):
        """
        main interaction sequence
        """

        #TODO need a start game pose

        #TODO need a rest pose for during the game

        self.play_game()

        #TODO need a game over for Draco's wins
        #TODO need a game over for Draco's loss
        #TODO end game pose


if __name__ == "__main__":
    connect = Connect4()
    connect.run()
