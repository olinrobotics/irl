#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String, Int16
import time
import pickle
import os, sys
import rospkg

class ArmBehaviors:
    def __init__(self):
        rospy.init_node('behavior_arm', anonymous=True)
        rospy.Subscriber('/behaviors_cmd', String, self.behavior_callback, queue_size=10)
        rospy.Subscriber('arm_status', Int16, self.status_callback, queue_size=10)
        self.pub = rospy.Publisher('/arm_cmd', String, queue_size=10)
        self.sound_pub = rospy.Publisher('/edwin_sound', String, queue_size=10)
        self.behaviors = {}
        self.moving = False

        self.create_behaviors()
        print "Starting behavior node"


    def status_callback(self, armstatus):
    	if armstatus == 1:
    		self.moving = True
    	else:
    		self.moving = False    

    def behavior_callback(self, cmdin):
        print "RECEIVED CMD: ", cmdin
        cmd = str(cmdin).replace("data: ", "")
        if cmd in self.behaviors.keys():

            cmd_list = self.behaviors[cmd].split(", ")
            for elem in cmd_list:
            	while self.moving == True:
            		continue
                if "R_" in elem:
                    msg = "data: run_route:: " + str(elem)
                elif "SPD" in elem:
                    msg = "data: set_speed:: " + str(elem.split("SPD: ")[1])
                else:
                    joint = elem.split(":")[0]
                    pos = elem.split(":")[1]
                    if joint == "H":
                        msg = "data: rotate_hand:: " + pos
                    elif joint == "WR":
                        msg = "data: rotate_wrist:: " + pos
                    elif joint == "E":
                        msg = "data: rotate_elbow:: " + pos
                    elif joint == "S":
                        msg = "data: rotate_shoulder:: " + pos
                    elif joint == "WA":
                        msg = "data: rotate_waist:: " + pos
                    elif joint == "SL":
                        msg = "data: sleeping:: " + pos


                print "Publishing: ", msg
                time.sleep(1)
                self.pub.publish(msg)

    def create_behaviors(self):
        """
        Need to add:
            -look_around
            -impatient
            -nudge
        """
        self.behaviors["impatient"] = "R_impat, SL: 1.5, R_ttt"
        self.behaviors["butt_wiggle"] = "WA: 500, WA: 1000, WA: 500, WA: 1000"
        self.behaviors["curiosity"] =  "R_curious, WR: 800, H: 0"
        self.behaviors["greet"] = "R_greet1, WR:1500, H: 100, H: 0"
        self.behaviors["sad"] = "H: 1000, WR: 1900, SPD: 300, R_sad_turn, SPD: 350, R_inhale, SL: .2, R_weep_top, SL: .2, R_weep_alittledown, SL: .2, R_weep_bottom, R_sigh_up, SL: 1, SPD: 500, R_sigh_down, SL: 1, SPD: 1000, R_look"
        self.behaviors["nudge"] = "R_look, R_nudge, R_look"
        self.behaviors["nod"] = "R_stare, E:13000, E:12000"
        self.behaviors["gloat"] = "H: 1000	, WR: 1700, SPD: 350, R_laugh, SPD: 500, R_pretentious_look, WR: 500, SL: 1, WR: 700, SL: 1, WR: 900, SL: 1, WR: 1100"
        self.behaviors["angry"] = "SPD: 200, R_stare, SPD: 1000"
        self.behaviors["sleep"] = "R_sleep"

        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")

        pickle.dump(self.behaviors, open(PACKAGE_PATH + '/params/behaviors.txt', 'wb'))


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    behavior_eng = ArmBehaviors()
    behavior_eng.run()
