#!/usr/bin/env python
import rospy
import rospkg
import math
import st
import numpy as np
from std_msgs.msg import String, Int16
import time
import pickle
import os, sys
import rospkg
from edwin.srv import arm_cmd


class ArmBehaviors:
    def __init__(self):
        rospy.init_node('behavior_arm', anonymous=True)
        rospy.Subscriber('/behaviors_cmd', String, self.behavior_callback, queue_size=10)
        self.arm_status = rospy.Publisher('/arm_status', String, queue_size=10)
        # self.pub = rospy.Publisher('/arm_cmd', String, queue_size=10)
        self.behaviors = {}
        self.moving = False
        self.serv_prob = False

        self.create_behaviors()
        print "Starting behavior node"

    def request_cmd(self, cmd):
        rospy.wait_for_service('arm_cmd', timeout=15)
        cmd_fnc = rospy.ServiceProxy('arm_cmd', arm_cmd)
        print "I have requested the command"

        try:
            resp1 = cmd_fnc(cmd)
            print "command done"


        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            self.arm_status.publish('error')
            self.serv_prob = True

    # def status_callback(self, armstatus):
    # 	if armstatus == 1:
    # 		self.moving = True
    # 	else:
    # 		self.moving = False

    def behavior_callback(self, cmdin):
        self.arm_status.publish('busy')
        self.serv_prob = False
        print "RECEIVED CMD: ", cmdin
        cmd = str(cmdin).replace("data: ", "")
        if cmd == "random":
            cmd = "impatient"
        elif "R_" in cmd:
            self.request_cmd("run_route:: "+cmd)
        elif cmd in self.behaviors.keys():
            cmd_list = self.behaviors[cmd].split(", ")
            for elem in cmd_list:
            	while self.moving == True:
            		continue
                if "R_" in elem:
                    msg = "data: run_route:: " + str(elem)
                elif "SPD" in elem:
                    msg = "data: set_speed:: " + str(elem.split("SPD: ")[1])
                elif "ACCEL" in elem:
                    msg = "data: set_accel:: " + str(elem.split("ACCEL: ")[1])
                else:
                    print "ELEM IS: ", elem
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
                    elif joint == "WA_rel":
                        msg = "data: rotate_waist_rel:: " + pos
                    elif joint == "SL":
                        msg = "no message"
                        # msg = "data: sleeping:: " + pos
                        time.sleep(float(pos))

                print "Publishing: ", msg
                self.request_cmd(msg)
        if not self.serv_prob:
            self.arm_status.publish('free')


                # if "R_" in elem:
                #     time.sleep(2.5)
                # else:
                #     time.sleep(1)


    def create_behaviors(self):

        ###################               General actions           #####################

        self.behaviors["happy_butt_wiggle"] = "R_curl_up, WA: 4500, WA: 5400, WA: 4500, WA: 5400, WA: 4500, WA: 5400, SL: .5, R_look"
        self.behaviors["curiosity"] =  "R_curious, WR: 800, H: 0"
        self.behaviors["greet"] = "R_greet1, WR:1500, H: 100, H: 0"
        self.behaviors["nudge"] = "R_look, H:0, H:-200, R_look"
        self.behaviors["nod"] = "R_stare, E:13000, E:12000"
        self.behaviors["gloat"] = "H: 1000	, WR: 1700, SPD: 350, R_laugh, SPD: 500, R_pretentious_look, WR: 500, SL: 1, WR: 700, SL: 1, WR: 900, SL: 1, WR: 1100"
        self.behaviors["angry"] = "SPD: 200, R_stare, SPD: 1000"
        self.behaviors["sleep"] = "R_sleep"
        self.behaviors["laugh"] = "SPD: 700, R_laugh, SPD: 1000"
        self.behaviors["idle_look_distance"] = "R_look_distance, SL: 1, WR: 1500, SL: .5, WR: 2300, R_look"
        self.behaviors["idle_sniff"] = "R_1_sniff, SPD: 1000, R_2_sniff, R_look"
        self.behaviors["idle_yawn"] = "SPD: 500, R_yawn, SL: .5, SPD: 500, R_slouch, SL: 1, R_look"
        self.behaviors["idle_butt_wiggle"] = "R_scrunch_up, WA: 1250, WA: 750, WA: 1250, WA: 750"
        self.behaviors["idle_1_lookaround"] = "R_1_lookaround"
        self.behaviors["idle_2_lookaround"] = "H: 0, R_2_lookaround"
        self.behaviors["idle_3_lookaround"] = "R_3_lookaround, WR: 0, SL: .5, WR: 800"
        self.behaviors["idle_head_bobble"] = "R_head_bobble, R_look"
        self.behaviors["idle_wander"] = "R_squirrel, SL: 1, SPD: 300, R_follow_squirrel, SL: 1, SPD: 500, R_look"
        self.behaviors["idle_spin"] = "R_spin_position, WA: -17000, WA: 17000, WA: 5000, R_look"
        self.behaviors["pout"] = "R_sad_turn, WA: -1000, WA: 1000"
        self.behaviors["impatient"] = "WA: 250, WA: -250, WA: 250, WA: -250, WA: 0, SPD: 700, R_impatient, SL: 1, R_annoyed_nudge"
        self.behaviors["bored"] = "SPD: 400, R_bored, R_stare_away"

        ###################               Simon Says actions           #####################
        self.behaviors['touch_head'] = "SPD: 600, R_touch_head "
        self.behaviors['high5_self'] = "R_high5_self "
        self.behaviors['hug_self'] = "SPD: 600, R_hug_self"
        self.behaviors['dab'] = "R_dab"
        self.behaviors['starfish'] = "SPD: 700, R_starfish"
        self.behaviors['bow'] = "SPD: 500, R_bow "
        self.behaviors['heart'] = "SPD: 700, R_heart"
        self.behaviors['high5_self'] = "SPD: 800, R_high5_self"
        self.behaviors['wave'] = "R_wave"
        self.behaviors['disco'] = "SPD: 800, R_disco"
        self.behaviors['rub_tummy'] = "R_rub_tummy"


        self.behaviors["gloat"] = "H: 1000	, WR: 1700, SPD: 400, R_laugh "
        self.behaviors["done_game"] = "SPD: 600, R_done_game"
        self.behaviors["get_set"] = "SPD: 600, R_get_set"
        self.behaviors["leader"] = "R_leader"
        self.behaviors["look"] = "R_look"
        self.behaviors["sad"] = "SPD: 400, R_sad"
        self.behaviors["praise"] = "SPD: 1000, R_praise"

        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")

        pickle.dump(self.behaviors, open(PACKAGE_PATH + '/params/behaviors.txt', 'w'))

    def loop_all(self):
        for key in self.behaviors.keys():
            print " "
            print "--------"
            print "RUNNING: ", key
            print "--------"
            print " "
            self.behavior_callback(key)
            time.sleep(5)

    def run_once(self, key):
        time.sleep(2)
        print " "
        print "--------"
        print "RUNNING: ", key
        print "--------"
        print " "
        self.behavior_callback(key)
        time.sleep(2)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    """
    To fix:
    nudge
    idle_spin
    impatient
    idle_head_bobble
    """
    behavior_eng = ArmBehaviors()
    behavior_eng.run()
