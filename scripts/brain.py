#!/usr/bin/env python

"""
To run all the necessary components for the brain_eng

rosrun edwin arm_node.py
rosrun edwin arm_behaviors.py
rosrun edwin draw.py
rosrun edwin edwin_audio.py
rosrun edwin soundboard.py
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB1 _baud:=9600
"""

import rospy
import random
import math
import time
import numpy as np
from std_msgs.msg import String
import pickle, os, sys

from InteractiveDemos import TicTacToe as ttt

class EdwinBrain:
    def __init__(self):
        rospy.init_node('edwin_brain', anonymous=True)
        rospy.Subscriber('/edwin_sound', String, self.sound_callback, queue_size=1)
        rospy.Subscriber('/edwin_imu', String, self.imu_callback, queue_size=1)
        rospy.Subscriber('/edwin_decoded_speech', String, self.speech_callback, queue_size=1)

        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=2)
        self.behav_pub = rospy.Publisher('/behaviors_cmd', String, queue_size=2)
        self.emotion_pub = rospy.Publisher('/edwin_emotion', String, queue_size=2)
        self.idle_pub = rospy.Publisher('/idle_cmd', String, queue_size=2)

        self.idling = True
        self.behaviors = {}
        self.create_behaviors()
        self.categorized_behaviors = {}
        self.categorize_behaviors()


    def create_behaviors(self):
        curr_dir = os.path.dirname(os.path.realpath(__file__))
        if os.path.exists(curr_dir+'/storage.txt'):
            self.behaviors = pickle.load(open(curr_dir+'/storage.txt', 'rb')) 
    def categorize_behaviors(self):
        categorized_behaviors['negative_emotions'] = ['angry', 'sad']
        categorized_behaviors['happy_emotions'] = ['nod', 'butt_wiggle']
        categorized_behaviors['greeting'] = ['greet', 'nudge', 'curiosity']
        categorized_behaviors['pretentious']  = ['gloat'] 
        categorized_behaviors['calm'] = ['sleep', 'nudge', 'nod']    
    

    def speech_callback(self, data):
        """
        Generates response based on speech input
        """
        self.idle_pub.publish("stop_idle")
        speech = data.data
        print "RECEIVED SPEECH: ", speech

        if "hello" in speech:
            self.behav_pub.publish(random.choice(categorized_behaviors['greeting']))
        elif "happy" in speech:
            self.behav_pub.publish(random.choice(categorized_behaviors['happy_emotions']))
        elif "game" in speech:
            self.start_game = "TTT"
        elif "goodbye" in speech:
            self.behav_pub(random.choice(categorized_behaviors['calm']))
            self.idle_pub("go_idle")
    def sound_callback(self, data):
        """
        format in "byte_length peak_volume"
        """
        print "heard a loud noise!"
        print data
        self.behav_pub.publish("sleep")
        self.emotion_pub.publish("STARTLE")

    def imu_callback(self, data):
        """
        IMU: no touch/patted/slapped
        """
        state = data.data.replace("IMU: ", "")
        #print "STATE IS: ", state
        if state == "notouch":
            return
        elif state == "pat":
            if self.idling:
                self.start_game = "TTT"
                self.idling = False
                self.idle_pub.publish("stop_idle")
        elif state == "slap":
            emote_msg = "ANGRY"
            behav_msg = random.choice(categorized_behaviors['negative_emotions'])

        self.behav_pub.publish(behav_msg)
        self.emotion_pub.publish(emote_msg)
        time.sleep(10)

    def run_game(self):
        if self.start_game == "TTT":
            ttt_gm = ttt.Game()
            ttt_gm.run()
            self.idle_pub.publish("go_idle")
            self.idling = True

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.start_game != None:
                self.run_game()
            r.sleep()

if __name__ == '__main__':
    brain_eng = EdwinBrain()
    brain_eng.run()
