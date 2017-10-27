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


class AI_Player(object):

    def __init__(self, n_actions, memory, reset, player_type):
        self.player_type = player_type
        self.observation = None
        self.observation_ = None
        self.action = None
        self.reward = None
        self.memory = memory
        self.lut = QLearningTable(actions=list(range(n_actions)),
                                q_table= None if reset else self.load_q_table())


    def load_q_table1(self):
        return pd.read_pickle(self.memory)

    def store_memory(self):
        self.lut.q_table.to_pickle(self.memory)

    def set_observation(self, observation):
        self.observation = observation

    def set_observation_(self, observation_):
        self.observation_ = observation_

    def set_reward(self, reward):
        self.reward = reward

    def choose_action(self):
        self.action = self.lut.choose_action(str(self.observation))

    def learn(self):
        self.lut.learn(str(self.observation), self.action, self.reward, str(self.observation_))


class Reinforce(object):

    def __init__(self, train, reset):
        self.env = C4Board(self.render)
        self.train = train
        self.RL1 = AI_Player(self.env.n_actions, '/memory/player1.txt', reset, 1)
        self.RL2 = AI_Player(self.env.n_actions, '/memory/player2.txt', reset, 2)


    def run(self):
        if self.train:
            self.train()
        else:
            self.play()

#TODO FIX PLAY
    # def play(self):
    #
    #     if np.random.choice([1,2]) == 1:
    #         print " I WILL GO FIRST"
    #         self.ai_move(observation)
    #     observation = self.env.reset()
    #
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
            self.RL1.set_observation(self.env.reset())
            print "EPISODE", episode
            while True:

                # RL choose action based on observation
                self.RL1.choose_action()

                # RL take action and get next observation and reward
                observation1_, reward1, reward2, done = self.env.step(self.RL1.action, 1)

                self.RL1.set_observation_(observation1_)
                self.RL1.set_reward(reward1)
                self.RL2.set_reward(reward2)

                if done:
                    self.RL1.learn()
                    self.RL2.learn()
                    print "GAME ENDED"
                    break

                else:
                    if self.RL2.action2:
                        self.RL2.learn()

                self.RL2.set_observation(self.RL1.observation1_)

                # RL choose action based on observation
                self.RL2.choose_action()

                # RL take action and get next observation and reward
                observation2_, reward1, reward2, done = self.env.step(self.RL2.action, 2)

                self.RL2.set_observation_(observation2_)
                self.RL2.set_reward(reward2)
                self.RL1.set_reward(reward1)

                if done:
                    self.RL1.learn()
                    self.RL2.learn()
                    print "GAME ENDED"
                    break
                else:
                    self.RL1.learn()

                self.RL1.set_observation(self.RL2.observation2_)


        print "TRAINING OVER"
        self.RL1.store_memory()
        self.RL2.store_memory()
        print "MEMORY STORED, SESSION FINISHED"


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--train', action='store_true')
    parser.add_argument('-r', '--reset', action='store_true')
    parser.add_argument('-v', '--render', action='store_true')
    args = parser.parse_args()

    connect = Reinforce(args.train, args.reset, args.render)
    connect.run()
