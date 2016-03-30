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

        self.idling = True
        self.moving = False

        self.pat = False
        self.slap = True

    def arm_mvmt_callback(self, data):
        if data.data == 1:
            self.moving = True
        elif data.data == 0:
            self.moving = False

    def speech_callback(self, data):
        """
        Generates response based on speech input
        """
        self.idle_pub.publish("stop_idle")
        speech = data.data
        print "RECEIVED SPEECH: ", speech
        if "hello" in speech:
            self.behav_pub.publish("greet")
        elif "happy" in speech:
            self.behav_pub.publish("nod")
        elif "game" in speech:
            self.start_game = "TTT"
        elif "bye" in speech:
            self.behav_pub("butt_wiggle")
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
        IMU: no patted/slapped
        """
        state = data.data.replace("IMU: ", "")
        #print "STATE IS: ", state
        if self.moving == False:
            if state == "pat":
                self.pat = True
                self.slap = False
                self.behav_pub.publish("nod")
                time.sleep(5)
                # if self.idling:
                #     self.start_game = "TTT"
                #     self.idling = False
                #     self.idle_pub.publish("stop_idle")
            elif state == "slap":
                emote_msg = "ANGRY"
                behav_msg = "sleep"

                self.behav_pub.publish(behav_msg)
                self.emotion_pub.publish(emote_msg)
                time.sleep(5)

    def run_game(self):
        if self.start_game == "TTT":
            ttt_gm = ttt.Game()
            ttt_gm.run()
            self.idle_pub.publish("go_idle")
            self.idling = True

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # if self.start_game != None:
            #     self.run_game()
            r.sleep()

if __name__ == '__main__':
    brain_eng = EdwinBrain()
    brain_eng.run()
