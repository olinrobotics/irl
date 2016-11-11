#!/usr/bin/env python
import rospy
import rospkg
import random
import math
import st
import numpy as np
from std_msgs.msg import String
import time
import pickle, os, sys

class IdleBehaviors:
    def __init__(self):
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        rospy.init_node('idle', anonymous=True)
        rospy.Subscriber('/behaviors_cmd', String, self.callback, queue_size=10)
        rospy.Subscriber('/arm_cmd', String, self.callback, queue_size=10)
        rospy.Subscriber('/all_control', String, self.control_callback, queue_size=10)

        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=10)
        self.behav_pub = rospy.Publisher('/behaviors_cmd', String, queue_size=10)

        self.last_interaction = time.time()
        self.stop_idle_time = time.time()

        self.idling = False
        self.idle_time = random.randint(5, 7)
        print "Starting idle node"

    def control_callback(self, data):
        print "IDLE CMD: ", data.data
        if "idle stop" in data.data:
            self.idling = False
            self.stop_idle_time = time.time()
        elif "idle go" in data.data:
            self.idling = True
            msg = random.choice(self.idle_behaviors.keys())
            print "PUBLISHING: ", msg
            self.behav_pub.publish(msg)
            time.sleep(3)
        elif "idle init" in data.data:
            rospack = rospkg.RosPack()
            PACKAGE_PATH = rospack.get_path("edwin")

            self.routes = pickle.load(open(PACKAGE_PATH + '/params/routes.txt', 'rb'))
            self.idle_behaviors = pickle.load(open(PACKAGE_PATH + '/params/behaviors.txt', 'rb'))
            self.idle_behaviors = {key: value for key, value in self.idle_behaviors.items()
                 if "idle" in key}
            print self.idle_behaviors
            self.idling = True

    def callback(self, data):
        self.last_interaction = time.time()

    def run(self):
        r = rospy.Rate(10)
        joints = ["H", "WR", "E", "WA", "BEHAV"]
        while not rospy.is_shutdown():
            if int(time.time() - self.stop_idle_time) > 1000: #if we've stopped interacting for 15 mins, start again
                self.idling = True
            if self.idling:
                if int(time.time() - self.last_interaction) > self.idle_time:
                    self.idle_time = random.randint(5, 10)
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
                        msg = random.choice(self.idle_behaviors.keys())
                        print "PUBLISHING: ", msg
                        self.behav_pub.publish(msg)
                        time.sleep(3)
                    if joint != "BEHAV":
                        print "PUBLISHING: ", msg
                        self.arm_pub.publish(msg)
            # r.sleep()

if __name__ == '__main__':
    idle_eng = IdleBehaviors()
    idle_eng.run()
