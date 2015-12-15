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

        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=10)
        self.behav_pub = rospy.Publisher('/behaviors_cmd', String, queue_size=10)

        self.last_interaction = time.time()

        self.routes = ["R_look", "R_playful", "R_sleep", "R_wakeup", "R_leaving, R_greet1", "R_curious"]
        self.behaviors = {}
        self.create_behaviors()

    def callback(self, data):
        self.last_interaction = time.time()

    def create_behaviors(self):
        self.behaviors["butt_wiggle"] = "R_leaving, WA: 1000, WA: 800, WA: 1000"
        self.behaviors["curiosity"] =  "R_curious, H: 0, WR: 800, H: 100, WR: 2000"
        self.behaviors["greet"] = "R_greet1, WR:1500, H: 100, H: 0, H: 300, H: 100"
        self.behaviors["sleep"] = "R_sleep, H: 700, R: 1000"

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if time.time() - self.last_interaction > random.randint(5, 10):
                print "running random"
                i = self.routes[random.randint(0, len(self.routes)-1)]
                msg = "data: run_route:: " + str(i)
                self.arm_pub.publish(msg)
            r.sleep()

if __name__ == '__main__':
    idle_eng = IdleBehaviors()
    idle_eng.run()
    rospy.spin()