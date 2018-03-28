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


class Main(object):

    def __init__(self):
        self.env = RL_environment()
        self.RL = RL_brain()
        self.accuracy = 0
        self.trials = 1000
        self.trial_finished = False
        self.observation = None
        self.action = None
        self.reward = None
        self.observation_ = None
        self.test_interval = 100

    def run(self):

        for i in range(self.trials):
            self.observation = self.env.reset()
            self.trial_finished = False
            print "EPISODE", i
            while not self.trial_finished:

                self.action = self.RL.choose_action(str(self.observation))

                self.observation_, self.reward, self.trial_finished = self.env.step(self.action)

                self.RL.learn(str(self.observation, self.action, self.reward, self.observation_))

            if i%self.test_interval == 0:
                for _ in range(10):
                    self.observation = self.env.reset()
                    self.trial_finished = False
                    while not self.trial_finished:
                        self.action = self.RL.choose_action(str(self.observation))
                        self.observation_, self.reward, self.trial_finished = self.env.step(self.action)
                    self.accuracy += 1 if self.reward == 1 else 0
                print "TEST ACCRUACY", self.accuracy /= 10



        print "FINISHED TRAINING"


if __name__ == "__main__":
    main = Main()
    main.run()
