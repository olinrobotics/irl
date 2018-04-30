#!/usr/bin/env python


import rospy
import numpy as np
from std_msgs.msg import String, Int16
import time
from irl.msg import Cube, Structure
from cube import Digital_Cube
from RL import RL_brain
import argparse
import itertools
import cPickle as pickle


class Smart_Sequencer(object):

    def __init__(self):
        print "LOADING MEMORY"
        with open('/home/rooster/catkin_ws/src/memory/final_memory.txt', 'rb') as f:
            q_table = pickle.load(f)
        print "DONE LOADING"
        self.RL = RL_brain(q_table=q_table)
        self.observation = None
        self.action = None
        self.reward = None
        self.env = None
        self.agent_state = [0]*(self.num_actions*2)
        self.sequence = []
        self.one_hot_mapping = {"[1, 1]":0,  "[2, 1]":1,  "[3, 1]":2, \
                                "[1, 2]":3,  "[2, 2]":4,  "[3, 2]":5, \
                                "[1, 3]":6,  "[2, 3]":7,  "[3, 3]":8}
        self.reverse_mapping = {0:[1, 1],  1:[2, 1],  2:[3, 1], \
                                3:[1, 2],  4:[2, 2],  5:[3, 2], \
                                6:[1, 3],  7:[2, 3],  8:[3, 3]}


    def convert_to_onehot(self, structure):
        self.agent_state = [0]*(self.num_actions*2)
        self.sequence = []
        for cube in structure:
            one_hot = self.one_hot_mapping[str([cube.x, cube.y])]
            self.agent_state[one_hot] = 1
            self.agent_state[one_hot+self.num_actions] = 1
        return self.agent_state[:]

    def find_cube(self, action, cubes):
        x, y = self.reverse_mapping(action)
        for cube in cubes:
            if cube.x == x and cube.y == y:
                return cube

        return None

    def step(self, action):
        self.agent_state[action.astype(int)] = 0
        s_ = self.agent_state[:]

        done = True if np.sum(self.agent_state[:9]) == 0 else False

        return s_, done



    def smart_sequence(self, environment):
        cubes = environment
        self.observation = self.convert_to_onehot(environment)
        done_sequencing = False
        while not done_sequencing:
            self.action = self.RL.choose_action(str(self.observation))
            self.sequence.append(self.find_cube(self.action, cubes))
            self.observation, done_sequencing = self.step(self.action)

        return self.sequence



if __name__ == "__main__":
    main = Smart_Sequencer()
    main.run()
