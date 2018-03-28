#!/usr/bin/env python


import rospy
import numpy as np
from std_msgs.msg import String, Int16
import time
from irl.msg import Cube, Structure
from cube import Digital_Cube
import itertools


class RL_environment(object):
