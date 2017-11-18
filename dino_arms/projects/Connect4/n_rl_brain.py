#!/usr/bin/env python

import numpy as np
import random
import time

class QLearningTable:
    def __init__(self, actions, learning_rate=0.004, reward_decay=0.9, e_greedy=0.95, q_table=None):
        self.actions = actions  # a list
        self.lr = learning_rate
        self.gamma = reward_decay
        self.epsilon = e_greedy
        self.q_table = {} if q_table is None else q_table

    def choose_action(self, observation):
        self.check_state_exist(observation)
        # action selection
        if np.random.uniform() > self.epsilon:
            # choose best action
            state_action = self.q_table[observation]
            state_action = np.argwhere(state_action==np.amax(state_action)).flatten().tolist()
            action = np.random.choice(state_action)
        else:
            # choose random action
            action = np.random.choice(self.actions)
        return action

    def learn(self, s, a, r, s_):
        self.check_state_exist(s)
        self.check_state_exist(s_)
        q_predict = self.q_table[s][a]
        q_target = r + self.gamma * np.amax(self.q_table[s_])
        self.q_table[s][a] += self.lr * (q_target - q_predict)  # update

    def update_params(self):
        self.lr *= (1-self.lr *.13)
        self.epsilon *= (1-self.epsilon*.005)

    def check_state_exist(self, state):
        if state not in self.q_table:
            # append new state to q table
            self.q_table[state] = np.zeros(len(self.actions))
