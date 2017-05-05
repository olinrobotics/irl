#!/usr/bin/env python

"""
This is the brain for Edwin Spring 2017

Integrates SimonSays and Homework Solver into a cohesive demo

To run all the necessary components for the brain.py

Please run:

rosrun edwin idle.py

For SimonSays:
-all skeleton stuff (roslaunch, rviz, skeleton.py, presence_detection, minimal launch)
-tts and stt simon

For Homework Solver:
-arm_write.py
-roslaunch usb_cam usb_camera.launch
-handwriting_recognition.py

How it works:
1. While nothing is happening, be idle and do idle things
2. When a person is detected with skeleton, presence_detection kicks in and finds the person
the person will be followed for this duration
3. The person will enter visual menu, calibrate, and then select a mode
4. After choosing a demo, the demo will execute and the person will do the demo
5. The demo is over, and now Edwin will re-enter Step 1

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
from sight.math_interp import Calculator
from sight.handwriting_recognition import HandwritingRecognition
from Interactions.visualmenu import VisualMenu


class EdwinBrain:
    def __init__(self):
        rospy.init_node('edwin_brain', anonymous=True)

        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=2)
        self.behav_pub = rospy.Publisher('/behaviors_cmd', String, queue_size=2)
        self.idle_pub = rospy.Publisher('/idle_cmd', Stringm queue_size=10)

        self.idling = False
        self.exit = False #should be catch all to exit all long running commands
        self.activity = None

        time.sleep(1)
        print "edwin brain is running"

        #visualmenu stuff
        self.menu = VisualMenu()
        self.menu_choice = None

        #homework stuff
        self.hand_recog = HandwritingRecognition()
        self.hand_recog.run()


        #idle starts now
        self.idle_pub.publish("idle:init")


    def demo(self):

        self.menu_choice = self.menu.run()
        self.idle_pub.publish("idle:stop")
        if self.menu_choice == "SimonSays: Simon":
            game = EdwinSimon()
            game.run()
        elif self.menu_choice == "SimonSays: Player":
            difficulty = raw_input("How good should Edwin be?\n")
    		game = EdwinPlayer(difficulty)
            game.run()
        elif self.menu_choice == "Homework":
            game = Calculator()
            game.run()

        time.sleep(3)
        self.idle_pub.publish("idle:go")


    def chicken(self):
        """
        Main loop that takes in demo???
        """
        


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()


if __name__ == '__main__':
    TheBrain = EdwinBrain()
    TheBrain.demo()
