#!/usr/bin/env python
"""
This is the script for the Brain. It requires theses modules to be running:

roslaunch skeleton_markers markers_from_tf.launch
roscd skeleton_markers
rosrun rviz rviz markers_from_tf.rviz

AND A WHOLE LOT MOREEEE
"""

import rospy
import cv2
import cv2.cv as cv
import numpy as np
import random
import time
import math
from collections import namedtuple
from std_msgs.msg import String
from sensor_msgs.msg import Image
from edwin.msg import Edwin_Shape, Bones
from cv_bridge import CvBridge, CvBridgeError


#################3#A WHOLE LOT MORE IMPORTS


class TheBrain(object):

    def __init__(self):
        rospy.init_node("The Brain")




    def run(self):
        ###Working on iiiiiiiiiiiiiiiiiiiittt"








if __name__ == "__main__":
    brain = TheBrain()
    brain.run()
