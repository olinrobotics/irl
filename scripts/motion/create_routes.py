#!/usr/bin/env python
import rospy
import random
import math
import time
import numpy as np
from std_msgs.msg import String, Int16
import rospkg
import pickle
import os, sys

class RouteCreator:
    def __init__(self):
        rospy.init_node('route_creator', anonymous=True)
        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=2)

        time.sleep(2)
        print "Initializing"

    def create(self):
        r1 = "R_mv2; 3296, 2308, 999, 0, 0, 0, 200, 2400, 1800, 720, 240, 2, 3296, 2308, 999, 0, 0, 0"
        r2 = "R_look; 2500, 2500, 2000, 186, 240, 21"
        r3 = "R_impat; 0, 3621, 4860, 545, 120, 21, 576, 3574, 4860, 275, 120, 21, 0, 3621, 4860, 545, 120, 21"
        r4 = "R_laugh; 1000, 700, 7000, -456, 150, 21, 2000, 2000, 2000, 580, 90, 21, 1500, 1500, 3500, 338, 90, 21, 1700, 1700, 3200, 338, 90, 21, 1500, 1500, 3500, 338, 90, 21, 1700, 1700, 3200, 338, 90, 21, 1500, 1500, 3500, 338, 90, 21, 1700, 1700, 3200, 338, 90, 21"
        r5 = "R_pretentious_look; 2000, 2000, 1300, 165, 90, 21"
        r6 = "R_ttt; 200, 2400, 1800, 720, 240, 21"
        r7 = "R_nudge; 2200, 2200, 1700, 630, 270, 21, 2700, 2700, 2200, 57, 240, 21, 2400, 2400, 1700, 569, 270, 21, 2900, 2900, 2200, 22, 270, 21"
        r8 = "R_sad_turn; 1000, -2700, 200, 930, 270, 21"
        r9 = "R_inhale; 1000, -2700, 1000, 547, 300, 21"
        r10 = "R_1_weep; 1000, -2700, 400, 739, 150, 21"
        r11 = "R_2_weep; 1000, -2700, 300, 819, 150, 21"
        r12 = "R_3_weep; 1000, -2700, 200, 870, 150, 21"
        r13 = "R_4_weep; 1000, -2700, 100, 950, 120, 21"
        r14 = "R_5_weep; 1000, -2700, 0, 1030, 120, 21"
        r15 = "R_sigh_up; 1000, -3000, 1700, 209, 180, 21"
        r16 = "R_sigh_down; 1000, -2500, 0, 940, 165, 21"
        r17 = "R_curl_up; 1573, 1574, 1262, 760, 150, 21"
        r18 = "R_look_distance; 1000, -2500, 6000, 80, 240, 21"
        r19 = "R_1_sniff; 1000, 4000, -700, 358, 240, 21, 1000, 4000, -700, 298, 270, 21, 1000, 4000, -700, 358, 240, 21, 1000, 4000, -700, 298, 270, 21,  1000, 4000, -700, 358, 240, 21"
        r20 = "R_2_sniff; 4000, -100, -500, 346, 240, 21, 4000, -100, -500, 286, 240, 21, 4000, -100, -500, 346, 240, 21, 4000, -100, -500, 286, 240, 21, 4000, -100, -500, 346, 240, 21"
        r21 = "R_yawn; 1100, 1100, 7000, -850, 210, 21"
        r22 = "R_slouch; 2400, 2400, 1000, 602, 210, 21, 2400, 2400, 1000, 151, 240, 21"
        r23 = "R_scrunch_up; 400, 3500, 300, 186, 240, 21"
        r24 = "R_1_lookaround; 4000, 1500, 3000, 185, 240, 21"
        r25 = "R_2_lookaround; 500, 4000, 2000, 185, 240, 21"
        r26 = "R_3_lookaround; 3000, 2000, 4000, -39, 240, 21"
        r27 = "R_head_bobble; 3600, -100, 3500, 187, 240, 21, 4000, -100, 5000, 310, 240, 21, 4000, -100, 2000, 117, 240, 21, 4000, -100, 5000, 310, 240, 21, 4000, -100, 2000, 117, 240, 21, 3600, -100, 3500, 187, 240, 21"
        r28 = "R_squirrel; 4500, -1200, 5000, -195, 240, 21"
        r29 = "R_follow_squirrel; 3000, 4000, 3000, 82, 240, 21"
        r30 = "R_spin_position; 1500, 1500, 800, 64, 240, 21"

        routes = [r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, r13, r14, r15, r16, r17, r18, r19, r20, r21, r22, r23, r24, r25, r26, r27, r28, r29, r30]

        for r in routes:
            msg = "create_route:: " + r
            print "Sending message: ", msg
            self.arm_pub.publish(msg)
            time.sleep(2)


        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")

        pickle.dump(routes, open(PACKAGE_PATH + '/params/routes.txt', 'wb'))

    def run(self):
        self.create()

if __name__ == '__main__':
    rc = RouteCreator()
    rc.run()
