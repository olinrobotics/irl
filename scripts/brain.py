"""
To run all the necessary components for the brain_eng

rosrun edwin arm_node.py
rosrun edwin arm_behaviors.py
rosrun edwin edwin_audio.py
rosrun edwin soundboard.py
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB1 _baud:=9600
"""

#!/usr/bin/env python
import rospy
import random
import math
import time
import numpy as np
from std_msgs.msg import String

class EdwinBrain:
    def __init__(self):
        rospy.init_node('edwin_brain', anonymous=True)
        rospy.Subscriber('/edwin_sound', String, self.sound_callback, queue_size=1)
        rospy.Subscriber('/edwin_imu', String, self.imu_callback, queue_size=1)

        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=2)
        self.behav_pub = rospy.Publisher('/behaviors_cmd', String, queue_size=2)
        self.emotion_pub = rospy.Publisher('/edwin_emotion', String, queue_size=2)

        self.routes = ["R_look", "R_playful", "R_sleep", "R_wakeup", "R_leaving, R_greet1", "R_curious"]
        self.behaviors = {}
        self.create_behaviors()

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
            emote_msg = "HAPPY"
            behav_msg = "butt_wiggle"
        elif state == "slap":
            emote_msg = "ANGRY"
            behav_msg = "sleep"

        self.behav_pub.publish(behav_msg)
        self.emotion_pub.publish(emote_msg)
        time.sleep(10)

    def create_behaviors(self):
        self.behaviors["butt_wiggle"] = "R_leaving, WA: 1000, WA: 800, WA: 1000"
        self.behaviors["curiosity"] =  "R_curious, H: 0, WR: 800, H: 100, WR: 2000"
        self.behaviors["greet"] = "R_greet1, WR:1500, H: 100, H: 0, H: 300, H: 100"
        self.behaviors["sleep"] = "R_sleep, H: 700, R: 1000"

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    brain_eng = EdwinBrain()
    brain_eng.run()
