#!/usr/bin/env python


import rospy
import numpy as np
from std_msgs.msg import String, Int16
import time
from irl.msg import Cube, Structure
from cube import Digital_Cube
from env_RL import RL_environment
from RL import RL_brain
import itertools
import cPickle as pickle


class Main(object):

    def __init__(self):
        self.env = RL_environment()
        self.RL = RL_brain()
        self.accuracy = 0
        self.trials = 100
        self.trial_finished = False
        self.observation = None
        self.action = None
        self.reward = None
        self.observation2 = None
        self.test_interval = 1000

    def run(self):

        for i in range(self.trials):
            self.observation = self.env.reset()
            self.trial_finished = False
            # print "OBESRVATION IN MAIN", self.observation
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
                accuracy = 0
                for _ in range(100):
                    self.observation = self.env.reset()
                    self.trial_finished = False
                    while not self.trial_finished:
                        self.action = self.RL.choose_action(str(self.observation))
                        self.observation, self.reward, self.trial_finished = self.env.step(self.action)
                    self.accuracy += 0 if self.reward == -1 else 1
                print "TEST ACCRUACY", self.accuracy / 100.0

        print "FINISHED TRAINING"
        print "THERE ARE", len(self.RL.q_table), "TOTAL STATES"
        with open('/home/rooster/catkin_ws/src/irl/dino_arms/projects/Planning/RL/memory.txt', 'wb') as f:
            pickle.dump(self.RL.q_table, f)

        print "MEMORY SAVED"

if __name__ == "__main__":
    main = Main()
    main.run()
