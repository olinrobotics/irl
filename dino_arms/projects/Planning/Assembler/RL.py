#!/usr/bin/env python

import rospy
import numpy as np
import pandas as pd
import time


class RL_brain(object):

    def __init__(self, q_table=None):
        self.encoded_action = None
        self.real_action = None
        self.q_table = {} if q_table is None else q_table

    def choose_action(self, observation):
        # action selection
        eval_func = eval("lambda: " + observation)
        state = eval_func()
        num_actions = np.count_nonzero(state[:9])
        try:
            state_action = self.q_table[observation]
            # print "STATE ACTION", state_action
        except KeyError:
            self.q_table[observation] = np.zeros(num_actions)
            state_action = self.q_table[observation]

        # choose best action
        state_action = np.argwhere(state_action==np.amax(state_action)).flatten().tolist()
        self.encoded_action = np.random.choice(state_action)

        self.real_action = np.nonzero(state)[0][self.encoded_action]

        return self.real_action
