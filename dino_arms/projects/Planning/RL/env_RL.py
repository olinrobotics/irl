#!/usr/bin/env python


import rospy
import numpy as np
from std_msgs.msg import String, Int16
import time
from irl.msg import Cube, Structure
from cube import Digital_Cube
from digital_env import Environment
import itertools


class RL_environment(object):


    def __init__(self):
        self.env = Environment()
        self.action_space = range(27)
        self.num_actions = len(self.action_space)
        self.target = None
        self.current_index = 0
        self.current_bin = []
        self.agent_state = np.zeros(27)
        self.one_hot_mapping = {"[0, 0, 0]":0,  "[0, 1, 0]":1,  "[0, 2, 0]":2, \
                                "[1, 0, 0]":3,  "[1, 1, 0]":4,  "[1, 2, 0]":5, \
                                "[2, 0, 0]":6,  "[2, 1, 0]":7,  "[2, 2, 0]":8, \
                                "[0, 0, 1]":9,  "[0, 1, 1]":10,  "[0, 2, 1]":11, \
                                "[1, 0, 1]":12,  "[1, 1, 1]":13,  "[1, 2, 1]":14, \
                                "[2, 0, 1]":15,  "[2, 1, 1]":16,  "[2, 2, 1]":17, \
                                "[0, 0, 2]":18,  "[0, 1, 2]":19,  "[0, 2, 2]":20, \
                                "[1, 0, 2]":21,  "[1, 1, 2]":22,  "[1, 2, 2]":23, \
                                "[2, 0, 2]":24,  "[2, 1, 2]":25,  "[2, 2, 2]":26}
        self.build_maze()

    def build_maze(self):
        self.reset()


    def reset(self):
        structure = self.env.create_a_struct()
        self.target = []
        for cube_bin in structure:
            target_bin = []
            for cube in cube_bin:
                one_hot = self.one_hot_mapping[str([cube.x, cube.y,cube.z])]
                target_bin.append(one_hot)
                self.agent_state[one_hot] = 1
            self.target.append(target_bin)
        return self.agent_state

    def step(self, action):
        s = self.agent_state
        self.agent_state[action] = 0
        s_ = self.agent_state

        if action not in self.target[self.current_index]:
            reward = -1
            done = True
            return s_, reward, done
        if action in self.target[self.current_index]:
            reward = 0
            done = False
            self.current_bin.append(action)
            if sorted(self.current_bin) == sorted(self.target.current_index):
                self.target = self.target[1:]
                self.current_bin = []
                self.current_index += 1

        if self.target == []:
            reward = 1
            done = True

        return s_, reward, done


if __name__=="__main__":
    test = RL_environment()
    # test.reset()
