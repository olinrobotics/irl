#!/usr/bin/env python


import rospy
import numpy as np
from std_msgs.msg import String, Int16
import time
from irl.msg import Cube, Structure
from cube import Digital_Cube
from env_RL import RL_environment
from RL import RL_brain
import argparse
import itertools
import cPickle as pickle
import matplotlib.pyplot as plt

"""
best parameters so far:
lr = .02, every million .9 it
epsilon = 1, every trial (1-epsilon*.0000002)

similar:
lr = .02, every million .99 it
epsilon = 1, every trial (1-epsilon*.0000002)

again similar:
lr adaptive
epsilon = 1, every trial (1-epsilon*.0000002)

even better: USING RIGHT NOW
lr adaptive
epsilon = 1, every trial (1-epsilon*.000003)

even worse:
lr adaptive
epsilon = 1, every trial (1-epsilon*.00000009)

epsilon with .000005 and .000009 also work, even stronger, but for 5x5 could be too exploitive
"""

class Main(object):

    def __init__(self, train):
        self.env = RL_environment()
        self.RL = None
        self.avg_reward = 0
        self.trials = 30000000
        self.trial_finished = False
        self.observation = None
        self.action = None
        self.reward = None
        self.observation2 = None
        self.test_interval = 1000
        self.mode = train


    def run(self):
        if self.mode:
            self.train()
        else:
            self.test()

    def test(self):
        print "LOADING MEMORY"

        with open('/home/rooster/catkin_ws/src/memory/memory30mil_new_context.txt', 'rb') as f:
            q_table = pickle.load(f)
        print "DONE"
        self.RL = RL_brain(e_greedy=0, q_table=q_table)

        print "------FIRST SHOWING TESTING ON 100\n"
        accuracy = 0
        for i in range(100):
            self.observation = self.env.reset()
            done_sequencing = False
            while not done_sequencing:
                self.action = self.RL.choose_action(str(self.observation))
                self.observation, self.reward, done_sequencing = self.env.step(self.action)
            accuracy += 0 if self.reward == -1 else 1
        print "TEST ACCURACY", accuracy, "%"

        print "------NOW SHOWING VERBOSE ON 10\n"

        for i in range(10):
            self.observation = self.env.reset()
            target = self.env.target[:]
            done_sequencing = False
            sequence = []
            while not done_sequencing:
                self.action = self.RL.choose_action(str(self.observation))
                sequence.append(self.action)
                self.observation, self.reward, done_sequencing = self.env.step(self.action)

            print "-----CORRECT SEQUENCE-----" if self.reward != -1 else "MISSED IT"
            print "RL's SEQUENCE", sequence
            print "TARGET's LIST", target



    def train(self):
        self.RL = RL_brain()
        reward_list = []

        for i in range(self.trials):
            self.observation = self.env.reset()

            self.trial_finished = False
            # print "OBESRVATION IN MAIN", self.observation
            if i%self.test_interval == 0:
                print "EPISODE", i
            while not self.trial_finished:

                # print "MAKING MOVE"

                # print "IN MAIN OBSERVATION BEFORE CHOOSE", str(self.observation)
                self.action = self.RL.choose_action(str(self.observation))

                self.observation2, self.reward, self.trial_finished = self.env.step(self.action)

                # print "IN MAIN OBSERVATION BEFORE LEARN ", str(self.observation)
                self.RL.learn(str(self.observation), self.reward, str(self.observation2))

                self.observation = self.observation2


            self.RL.update_params()
            # print 'Correct\n' if self.reward == 1 else "NOT\n"

            if i%self.test_interval == 0:
                self.avg_reward = 0
                for _ in range(100):
                    self.observation = self.env.reset()
                    self.trial_finished = False
                    while not self.trial_finished:
                        self.action = self.RL.choose_action(str(self.observation))
                        self.observation, self.reward, self.trial_finished = self.env.step(self.action)
                    self.avg_reward += self.reward
                self.avg_reward /= 100.0
                print "AVG. REWARD (per 100)", self.avg_reward, ""
                reward_list.append(self.avg_reward)

        print "FINISHED TRAINING"
        print "THERE ARE", len(self.RL.q_table), "TOTAL STATES"
        with open('/home/rooster/catkin_ws/src/memory/memory30mil_lradaptive_5x5.txt', 'wb') as f:
            pickle.dump(self.RL.q_table, f)

        print "MEMORY SAVED"
        plt.plot(range(self.trials/self.test_interval), reward_list)
        plt.axis([0,self.trials/self.test_interval, -1, 1])
        plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--train', action='store_true')
    args = parser.parse_args()

    main = Main(args.train)
    main.run()
