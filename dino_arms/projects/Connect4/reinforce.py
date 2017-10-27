#!/usr/bin/env python


import rospy
import numpy as np
import pandas as pd
import random
import time
from std_msgs.msg import String, Int16
import pickle
import argparse
from game_board import C4Board
from rl_brain import QLearningTable
from Queue import *


class Reinforce(object):

    def __init__(self, train, reset):
        self.env = C4Board(self.render)
        self.RL1 = QLearningTable(actions=list(range(self.env.n_actions)),
                                q_table= None if reset else self.load_q_table1())
        self.RL2 = QLearningTable(actions=list(range(self.env.n_actions)),
                                q_table= None if reset else self.load_q_table2())

        self.observation1 = None
        self.observation1_ = None
        self.observation2 = None
        self.observation2_ = None

        if train:
            self.train()
        else:
            self.play()


    def load_q_table1(self):
        return pd.read_pickle('/memory/player1.txt')
    def load_q_table2(self):
        return pd.read_pickle('/memory/player2.txt')

    def store_memory(self):
        self.RL1.q_table.to_pickle('/memory/player1.txt')
        self.RL2.q_table.to_pickle('/memory/player2.txt')


    def ai_move(self, observation):




    # def play(self):
    #     #TODO: FIX THIS THING
    #     observation = self.env.reset()
    #     if np.random.choice([1,2]) == 1:
    #         print " I WILL GO FIRST"
    #         self.ai_move(observation)
    #
    #     while True:
    #
    #         # visualize
    #         self.env.render()
    #
    #         action = int(raw_input("YOU CAN MOVE, WHAT'S YOUR MOVE?"))
    #
    #         observation, _, done = self.env.step(action, 2)
    #
    #         # break if I win
    #         if done:
    #             self.env.render()
    #             print "HUMAN WINS"
    #             break
    #
    #         done = self.ai_move(observation)
    #
    #         # break if ai wins
    #         if done:
    #             self.env.render()
    #             print "AI WINS"
    #             break
    #
    #     print "PLAYING OVER"
    #     self.store_memory()
    #     print "MEMORY STORED, TESTING FINISHED"


    def train(self):
        for episode in range(100000):
            # initial observation
            observation = self.env.reset()
            print "EPISODE", episode
            while True:

                # RL choose action based on observation
                action = self.RL1.choose_action(str(observation))

                # RL take action and get next observation and reward
                observation_, reward, done = self.env.step(action, 1)

                # RL learn from this transition
                self.RL1.learn(str(observation), action, reward, str(observation_))

                observation = observation_

                # break while loop when end of this episode
                if done:
                    print "GAME ENDED"
                    time.sleep(3)
                    break

                # RL choose action based on observation
                action = self.RL2.choose_action(str(observation))

                # RL take action and get next observation and reward
                observation_, reward, done = self.env.step(action, 2)

                # RL learn from this transition
                self.RL2.learn(str(observation), action, reward, str(observation_))

                # break while loop when end of this episode
                if done:
                    print "GAME ENDED"
                    time.sleep(3)
                    break

        print "TRAINING OVER"
        self.store_memory()
        print "MEMORY STORED, SESSION FINISHED"


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--train', action='store_true')
    parser.add_argument('-r', '--reset', action='store_true')
    parser.add_argument('-v', '--render', action='store_true')
    args = parser.parse_args()

    connect = Reinforce(args.train, args.reset, args.render)
