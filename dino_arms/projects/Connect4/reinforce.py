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

        self.obs_q = Queue()
        self.act_q = Queue()
        self.r_q = Queue()

        if train:
            self.train()
        else:
            self.play()


    def load_q_table1(self):
        return pd.read_pickle('player1.txt')
    def load_q_table2(self):
        return pd.read_pickle('player2.txt')

    def store_memory(self):
        self.RL1.q_table.to_pickle('player1.txt')
        self.RL2.q_table.to_pickle('player2.txt')


    def ai_move(self, observation):

        # visualize
        self.env.render()

        # RL choose action based on observation
        action = self.RL.choose_action(str(observation))

        # RL take action and get next observation and reward
        observation_, reward, done = self.env.step(action, 1)

        # RL learn from this transition
        self.RL.learn(str(observation), action, reward, str(observation_))

        if done:
            observation = self.env.next_turn()


        #TODO: FIX THIS WITH THE THINGY BELOW

        return done


    def play(self):
        #TODO: FIX THIS THING
        observation = self.env.reset()
        if np.random.choice([1,2]) == 1:
            print " I WILL GO FIRST"
            self.ai_move(observation)

        while True:

            # visualize
            self.env.render()

            action = int(raw_input("YOU CAN MOVE, WHAT'S YOUR MOVE?"))

            observation, _, done = self.env.step(action, 2)

            # break if I win
            if done:
                self.env.render()
                print "HUMAN WINS"
                break

            done = self.ai_move(observation)

            # break if ai wins
            if done:
                self.env.render()
                print "AI WINS"
                break

        print "PLAYING OVER"
        self.store_memory()
        print "MEMORY STORED, TESTING FINISHED"


    def train(self):
        for episode in range(100000):
            # initial observation
            observation = self.env.reset()
            self.obs_q.enqueue(observation)
            print "EPISODE", episode
            while True:
                # TODO: MAKE THIS WORK WITH REWARDS
                # done = self.ai_move(observation)
                # action = self.RL.choose_action(observation)
                # self.act_q.enqueue(action)

                observation = self.env.next_turn()

                # break while loop when end of this episode
                if done:
                    self.env.render()
                    print "GAME ENDED"
                    time.sleep(0.01)
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
