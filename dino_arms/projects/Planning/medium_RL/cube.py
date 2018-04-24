#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String, Int16
import time



class Digital_Cube(object):

    def __init__(self):
        self.height = 0
        self.connections = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.activated = False

    def turn_on(self, height, x, y, z):
        self.activated = True
        self.height = height
        self.x = x
        self.y = y
        self.z = z

    def set_connectivity(self, connectivity):
        self.connections = connectivity


    def turn_off(self):
        self.activated = False
        self.connections = 0
        self.height = 0
        self.x = 0
        self.y = 0
        self.z = 0
