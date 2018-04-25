#!/usr/bin/env python


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
        self.action_space = range(25)
        self.num_actions = len(self.action_space)
        self.target = None
        # self.current_bin = []
        self.agent_state = [0]*(self.num_actions*2)
        self.one_hot_mapping = {"[0, 0, 0]":0,  "[1, 0, 0]":1,  "[2, 0, 0]":2, "[3, 0, 0]":3, "[4, 0, 0]":4, \
                                "[0, 1, 0]":5,  "[1, 1, 0]":6,  "[2, 1, 0]":7, "[3, 1, 0]":8, "[4, 1, 0]":9, \
                                "[0, 2, 0]":10,  "[1, 2, 0]":11,  "[2, 2, 0]":12, "[3, 2, 0]":13, "[4, 2, 0]":14, \
                                "[0, 3, 0]":15,  "[1, 3, 0]":16,  "[2, 3, 0]":17, "[3, 3, 0]":18, "[4, 3, 0]":19, \
                                "[0, 4, 0]":20,  "[1, 4, 0]":21,  "[2, 4, 0]":22, "[3, 4, 0]":23, "[4, 4, 0]":24 }

    def build_maze(self):
        self.reset()


    def reset(self):
        structure = self.env.create_a_struct()
        self.target = []
        self.agent_state = [0]*(self.num_actions*2)
        for cube in structure:
            one_hot = self.one_hot_mapping[str([cube.x, cube.y,cube.z])]
            self.agent_state[one_hot] = 1
            self.agent_state[one_hot+self.num_actions] = 1
            self.target.append(one_hot)

        # print "\n\nSTARTING STATE", self.agent_state
        # print "THE TARGET", self.target
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
