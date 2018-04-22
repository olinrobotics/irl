#!/usr/bin/env python

import rospy
import numpy as np
import pandas as pd
import time


class RL_brain(object):

    def __init__(self, learning_rate=.02, reward_decay=0.9, e_greedy=1.0, q_table=None):
        self.encoded_action = None
        self.real_action = None
        self.lr = learning_rate
        self.gamma = reward_decay
        self.epsilon = e_greedy
        self.q_table = {} if q_table is None else q_table
        # self.iterations = 1
        self.q_table_counter = {}

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

        try:
            counters = self.q_table_counter[observation]
        except KeyError:
            self.q_table_counter[observation] = np.zeros(num_actions)


        if np.random.uniform() > self.epsilon:
            # choose best action
            state_action = np.argwhere(state_action==np.amax(state_action)).flatten().tolist()
            self.encoded_action = np.random.choice(state_action)
        else:
            # choose random action
            self.encoded_action = np.random.choice(np.arange(num_actions))

        self.real_action = np.nonzero(state)[0][self.encoded_action]

        self.q_table_counter[observation][self.encoded_action] += 1

        # print "IN CHOOSE ACTION", self.encoded_action
        # print "IN CHOOSE LENGTH OF QTABLE", len(self.q_table[observation])
        # print "IN CHOOSE OBSERVATION            ", observation
        # print "ENCODED", self.encoded_action
        # print "REAL", self.real_action

        return self.real_action


    def learn(self, s, r, s_):
        try:
            check = self.q_table[s]
        except KeyError:
            eval_func = eval("lambda: " + s)
            num_actions = np.count_nonzero(eval_func()[:9])
            self.q_table[s] = np.zeros(num_actions)

        try:
            counters = self.q_table_counter[s]
        except KeyError:
            eval_func = eval("lambda: " + s)
            num_actions = np.count_nonzero(eval_func()[:9])
            self.q_table_counter[s] = np.zeros(num_actions)

        try:
            check = self.q_table[s_]
        except KeyError:
            eval_func = eval("lambda: " + s_)
            num_actions = np.count_nonzero(eval_func()[:9])
            num_actions = num_actions if num_actions > 0 else 1
            self.q_table[s_] = np.zeros(num_actions)

        # print "IN LEARN", self.encoded_action
        # print "IN LEARN LENGTH OF QTABLE", len(self.q_table[s])
        # print "IN LEARN OBSERVATION", s
        # print "IN LEARN OBSERVATION2", s_
        # print "IN LEARN WHAT IS THIS", self.q_table[s_]

        q_predict = self.q_table[s][self.encoded_action]
        q_target = r + self.gamma * np.amax(self.q_table[s_])
        learning_rate = self.q_table_counter[s][self.encoded_action]
        # self.q_table[s][self.encoded_action] += self.lr * (q_target - q_predict)  # update
        self.q_table[s][self.encoded_action] += (1.0/learning_rate) * (q_target - q_predict)  # update



    def update_params(self):
        # if self.iterations % 1000000 == 0:
        #     # self.lr *= (1-self.lr *.000025)
        #     # self.lr *= 0.9
        #     self.lr *= 0.99
        # self.iterations += 1
        self.epsilon *= (1-self.epsilon*.00000009)
        # self.epsilon *= (1-self.epsilon*.0000009)
