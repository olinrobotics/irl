#!/usr/bin/env python

"""
The env_RL program is meant to act as a digital environment for the RL agent to run
through and helps decide the reward based on the actions taken by the agent.

This script is meant to be imported as a package into sequencer_RL.py

the basic functionality is as follows:

1. first initializes the environment, which consists of a one-hot mapping of the built
structure from digital_env.py, this can include resetting as well. also will
create a target which represents the sorted instructions as per the pattern sorter,
which is what we're trying to make the RL agent learn
2. after that, the sequencer_RL will call step on the environment given the action
the RL agent decided to take, which will then run the action through the environment
and return the results of running that action. this continues until the environment
decides

"""


import rospy
import numpy as np
from std_msgs.msg import String, Int16
import time
from irl.msg import Grid_Cube, Grid_Structure
from cube import Digital_Cube
from digital_env import Environment
import itertools


class RL_environment(object):


    def __init__(self):
        self.env = Environment()
        self.action_space = range(9)
        self.num_actions = len(self.action_space)
        self.target = None
        self.agent_state = [0]*(self.num_actions*2)
        self.one_hot_mapping = {"[0, 0]":0,  "[1, 0]":1,  "[2, 0]":2, \
                                "[0, 1]":3,  "[1, 1]":4,  "[2, 1]":5, \
                                "[0, 2]":6,  "[1, 2]":7,  "[2, 2]":8}

    def build_maze(self):
        self.reset()


    def reset(self):
        structure = self.env.create_a_struct()
        self.target = []
        self.agent_state = [0]*(self.num_actions*2)
        for cube in structure:
            one_hot = self.one_hot_mapping[str([cube.x, cube.y])]
            self.agent_state[one_hot] = 1
            self.agent_state[one_hot+self.num_actions] = 1
            self.target.append(one_hot)

        # print "\n\n\nTARGET", self.target
        # print "AGENT", self.agent_state
        return self.agent_state[:]


    def step(self, action):
        self.agent_state[action.astype(int)] = 0
        s_ = self.agent_state[:]

        if action != self.target[0]:
            reward = -1
            done = True
        elif action == self.target[0]:
            self.target = self.target[1:]
            reward = 1 if len(self.target) == 0 else 0
            done = True if len(self.target) == 0 else False

        return s_, reward, done


if __name__=="__main__":
    test = RL_environment()
    test.reset()
