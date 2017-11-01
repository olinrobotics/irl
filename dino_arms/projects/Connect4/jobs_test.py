#!/usr/bin/env python

import rospy
import numpy as np
import pandas as pd
import random
import time
from std_msgs.msg import String, Int16
import pickle
import argparse
from game_board import C4Board
from rl_brain import QLearningTable
from Queue import *
from joblib import Parallel, delayed


def massive_function():
    thing = []
    for i in range(100000000):
        thing.append(i)


    return thing


if __name__== "__main__":
    thing2 = Parallel(n_jobs=4)(delayed(massive_function)() for i in range(1000))
    print [item for item in thing2]
