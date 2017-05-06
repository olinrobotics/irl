#!/usr/bin/env python

"""
To run all the necessary components for the brain.py

Please run:

all skeleton stuff (roslaunch, rviz, skeleton.py, presence_detection, minimal launch)

tts and stt simon

and other stuff potentially


"""

import rospy
import rospkg
import random
import math
import time
import numpy as np
import pickle, os, sys
from std_msgs.msg import String, Int16


from Interactions.SimonSays import AutonomousSimon, EdwinSimon, EdwinPlayer
from sight.presence_detection import Presence

#TODO: get all the other imports

class EdwinBrain:
    def __init__(self):
        rospy.init_node('edwin_brain', anonymous=True)
        rospy.Subscriber('/vm_choice', String, self.visualmenu_callback, queue_size = 1)

        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=2)
        self.behav_pub = rospy.Publisher('/behaviors_cmd', String, queue_size=2)
        self.idle_pub = rospy.Publisher('/idle_cmd', Stringm queue_size=10)

        self.idling = False
        self.exit = False #should be catch all to exit all long running commands
        self.activity = None

        time.sleep(0.2)
        print "edwin brain is running"


        self.control_pub.publish("idle:init")


    def visualmenu_callback(self, data):
        try:
            self.command = data.data
        except data.data == None:
            pass


    def demo(self):
        while True:
            cmd = self.command
            self.control_pub.publish("idle:stop")
            if cmd == "sss":
                g = EdwinSimon()
                g.run()
            elif cmd == "ssp":
                difficulty = raw_input("How good should Edwin be?\n")
        		g = EdwinPlayer(difficulty)
                g.run()

            self.control_pub.publish("idle:go")


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()


if __name__ == '__main__':
    TheBrain = EdwinBrain()
    TheBrain.demo()
