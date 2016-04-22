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
import pickle, os, sys
from std_msgs.msg import String, Int16


from InteractiveDemos import TicTacToe as ttt

class EdwinBrain:
    def __init__(self):
        rospy.init_node('edwin_brain', anonymous=True)
        rospy.Subscriber('/edwin_sound', String, self.sound_callback, queue_size=1)
        rospy.Subscriber('/edwin_imu', String, self.imu_callback, queue_size=1)
        rospy.Subscriber('/edwin_decoded_speech', String, self.speech_callback, queue_size=1)
        rospy.Subscriber('/arm_status', Int16, self.arm_mvmt_callback, queue_size=1)

        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=2)
        self.behav_pub = rospy.Publisher('/behaviors_cmd', String, queue_size=2)
        self.emotion_pub = rospy.Publisher('/edwin_emotion', String, queue_size=2)
        self.idle_pub = rospy.Publisher('/idle_cmd', String, queue_size=2)
        self.control_pub = rospy.Publisher('/all_control', String, queue_size=2)

        self.idling = True
        self.moving = False

        self.pat = False
        self.slap = False
        self.ok = False

        self.exit = False #should be catch all to exit all long running commands
        self.start_game = None

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

    def arm_mvmt_callback(self, data):
        if data.data == 1:
            self.moving = True
        elif data.data == 0:
            self.moving = False

    def speech_callback(self, data):
        """
        Generates response based on speech input
        """
        speech = data.data
        print "RECEIVED SPEECH: ", speech
        if "hello" or "hi" in speech:
            if self.idling:
                self.idle_pub("stop_idle")
                self.control_pub("ft go")
            self.behav_pub.publish(random.choice(categorized_behaviors['greeting']))
        elif "game" in speech:
            self.start_game = "TTT"
        elif "bye" in speech:
            self.idle_pub("go_idle")
        elif "okay" in speech:
            self.ok = True

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
        IMU: no patted/slapped
        """
        state = data.data.replace("IMU: ", "")

        if self.moving == False:
            if state == "pat":
                self.pat = True
                self.slap = False
            elif state == "slap":
                emote_msg = "ANGRY"
                behav_msg = random.choice(categorized_behaviors['negative_emotions'])

                self.behav_pub.publish(behav_msg)
                self.emotion_pub.publish(emote_msg)
                time.sleep(5)

                self.pat = False
                self.slap = True

    def run_game(self):
        self.idle_pub.publish("stop_idle")
        self.idling = False

        if self.start_game == "TTT":
            self.ok = False
            self.behav_pub.publish("get_marker")
            start_marker_wait = time.time()
            while self.ok != True:
                #if Edwin has to wait too long for a marker, get impatient
                if int(start_marker_wait - time.time()) > 10:
                    self.behav_pub.publish("impatient")
                if self.slap:
                    self.behav_pub.publish("angry")
                elif self.exit:
                    self.exit = False
                    return
            self.ok = False
            time.sleep(5)
            ttt_gm = ttt.Game()
            ttt_gm.run()

        self.start_game = None
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
