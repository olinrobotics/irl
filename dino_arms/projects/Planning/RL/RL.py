#!/usr/bin/env python


import rospy
import numpy as np
from std_msgs.msg import String, Int16
import time
from irl.msg import Cube, Structure
from cube import Digital_Cube
import itertools


class RL_brain(object):

    def __init__(self):
        pass

    def choose_action(self, observation):
        pass

    def learn(self, observation, action, reward, observation_):
        pass

    def check_state_exist(self, state):
        pass
