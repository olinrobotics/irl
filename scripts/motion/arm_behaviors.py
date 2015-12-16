#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String
import time

class ArmBehaviors:
    def __init__(self):
        rospy.init_node('behavior_arm', anonymous=True)
        rospy.Subscriber('/behaviors_cmd', String, self.behavior_callback, queue_size=10)
        self.pub = rospy.Publisher('/arm_cmd', String, queue_size=10)

        self.routes = []
        self.behaviors = {}

        self.create_routes()
        self.create_behaviors()

    def create_routes(self):
        self.routes = ["R_look", "R_playful", "R_sleep", "R_wakeup", "R_leaving, R_greet1", "R_curious"]

    def behavior_callback(self, cmdin):
        cmd = str(cmdin).replace("data: ", "")
        if cmd in self.behaviors.keys():
            cmd_list = self.behaviors[cmd].split(", ")
            for elem in cmd_list:
                if "R_" in elem:
                    msg = "data: run_route:: " + str(elem)
                else:
                    joint = elem.split(":")[0]
                    pos = int(elem.split(":")[1])
                    if joint == "H":
                        msg = "data: rotate_hand:: " + str(pos)
                    elif joint == "WR":
                        msg = "data: rotate_wrist:: " + str(pos)
                    elif joint == "E":
                        msg = "data: rotate_elbow:: " + str(pos)
                    elif joint == "S":
                        msg = "data: rotate_shoulder:: " + str(pos)
                    elif joint == "WA":
                        msg = "data: rotate_waist:: " + str(pos)
                print "Publishing: ", msg
                self.pub.publish(msg)

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
    behavior_eng = ArmBehaviors()
    behavior_eng.run()
    rospy.spin()