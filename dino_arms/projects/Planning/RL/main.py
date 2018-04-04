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


class Main(object):

    def __init__(self, train):
        self.env = RL_environment()
        self.accuracy = 0
        self.trials = 10000000
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

        with open('/home/rooster/catkin_ws/src/irl/dino_arms/projects/Planning/RL/memory.txt', 'rb') as f:
            q_table = pickle.load(f)
        print "DONE"
        self.RL = RL_brain(e_greedy=0, q_table=q_table)

        print "------FIRST SHOWING TESTING ON 100\n"
        for i in range(100):
            self.observation = self.env.reset()
            done_sequencing = False
            accuracy = 0
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

            print "CORRECT SEQUENCE" if self.reward != -1 else "MISSED IT"
            print "RL's SEQUENCE", sequence
            print "TARGET's LIST", target



    def train(self):
        self.RL = RL_brain()

        for i in range(self.trials):
            self.observation = self.env.reset()
            self.trial_finished = False
            # print "OBESRVATION IN MAIN", self.observation
            if i%self.test_interval == 0:
                print "EPISODE", i
            while not self.trial_finished:

                # print "IN MAIN OBSERVATION BEFORE CHOOSE", str(self.observation)
                self.action = self.RL.choose_action(str(self.observation))

                self.observation2, self.reward, self.trial_finished = self.env.step(self.action)

                # print "IN MAIN OBSERVATION BEFORE LEARN ", str(self.observation)
                self.RL.learn(str(self.observation), self.reward, str(self.observation2))

                self.observation = self.observation2


            self.RL.update_params()

            if i%self.test_interval == 0:
                self.accuracy = 0
                for _ in range(100):
                    self.observation = self.env.reset()
                    self.trial_finished = False
                    while not self.trial_finished:
                        self.action = self.RL.choose_action(str(self.observation))
                        self.observation, self.reward, self.trial_finished = self.env.step(self.action)
                    self.accuracy += 0 if self.reward == -1 else 1
                print "TEST ACCURACY", self.accuracy, "%"

        print "FINISHED TRAINING"
        print "THERE ARE", len(self.RL.q_table), "TOTAL STATES"
        with open('/home/rooster/catkin_ws/src/irl/dino_arms/projects/Planning/RL/memory.txt', 'wb') as f:
            pickle.dump(self.RL.q_table, f)

        print "MEMORY SAVED"

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--train', action='store_true')
    args = parser.parse_args()

    main = Main(args.train)
    main.run()
