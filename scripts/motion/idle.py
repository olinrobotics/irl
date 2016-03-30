#!/usr/bin/env python
import rospy
import random
import math
import st
import numpy as np
from std_msgs.msg import String
import time

class IdleBehaviors:
    def __init__(self):
        rospy.init_node('idle', anonymous=True)
        rospy.Subscriber('/behaviors_cmd', String, self.callback, queue_size=10)
        rospy.Subscriber('/arm_cmd', String, self.callback, queue_size=10)
        rospy.Subscriber('/idle_cmd', String, self.idle_callback, queue_size=10)

        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=10)
        self.behav_pub = rospy.Publisher('/behaviors_cmd', String, queue_size=10)

        self.last_interaction = time.time()
        self.stop_idle_time = time.time()

        self.routes = ["R_look", "R_playful", "R_sleep", "R_wakeup", "R_leaving, R_greet1", "R_curious"]
        self.behaviors = {}
        self.create_behaviors()

        self.idling = True
        self.idle_time = random.randint(5, 10)
        print "Starting idle node"

    def idle_callback(self, data):
        print "IDLE CMD: ", data.data
        if "stop_idle" in data.data:
            self.idling = False
            self.stop_idle_time = time.time()
        elif "go_idle" in data.data:
            self.idling = True

    def callback(self, data):
        self.last_interaction = time.time()

    def create_behaviors(self):
        self.behaviors["butt_wiggle"] = "R_leaving, WA: 1000, WA: 800, WA: 1000"
        self.behaviors["curiosity"] =  "R_curious, H: 0, WR: 800, H: 100, WR: 2000"
        self.behaviors["greet"] = "R_greet1, WR:1500, H: 100, H: 0, H: 300, H: 100"
        self.behaviors["sleep"] = "R_sleep, H: 700, R: 1000"

    def run(self):
        r = rospy.Rate(10)
        joints = ["H", "WR", "E", "WA", "BEHAV"]
        while not rospy.is_shutdown():
            if int(time.time() - self.stop_idle_time) > 1500: #if we've stopped interacting for 20 mins, start again
                self.idling = True
            if self.idling:
                if int(time.time() - self.last_interaction) > self.idle_time:
                    self.idle_time = random.randint(3, 7)
                    print "IDLE"
                    joint = random.choice(joints)
                    if joint == "H":
                        msg = "data: rotate_hand:: " + str(random.randint(-900, 900))
                    elif joint == "WR":
                        msg = "data: rotate_wrist:: " + str(random.randint(-500, 2500))
                    elif joint == "E":
                        msg = "data: rotate_elbow:: " + str(random.randint(11000, 13000))
                    elif joint == "WA":
                        msg = "data: rotate_waist:: " + str(random.randint(4000, 6000))
                    elif joint == "BEHAV":
                        msg = random.choice(self.behaviors.keys())
                        print "PUBLISHING: ", msg
                        self.behav_pub.publish(msg)

                    if joint != "BEHAV":
                        print "PUBLISHING: ", msg
                        self.arm_pub.publish(msg)
            # r.sleep()

if __name__ == '__main__':
    idle_eng = IdleBehaviors()
    idle_eng.run()
