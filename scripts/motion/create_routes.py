#!/usr/bin/env python
import rospy
import random
import math
import time
import numpy as np
from std_msgs.msg import String, Int16

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
        r9 = "R_inhale; 1000, -2700, 600, 547, 300, 21"
        r10 = "R_weep_top; 1000, -2700, 400, 629, 240, 21"
        r11 = "R_weep_alittledown; 1000, -2700, 200, 709, 180, 21"
        r12 = "R_weep_bottom; 1000, -2700, 0, 790, 150, 21"
        r13 = "R_sigh_up; 1000, -3000, 1500, 514, 300, 21"
        r14 = "R_sigh_down; 1000, -2500, 0, 987, 180, 21"

        # r4 = "R_nudge2; 1942, 2609, 1854, 728, 240, 21"

        # self.arm.create_route("R_stare", [[3296, 2308, 999, 0, 0, 0]])
        # self.arm.create_route("R_ttt", [[200, 2400, 1800, 720, 240, 2.1]])
        # self.arm.create_route("R_look", [[3664, 1774, 3013, 11, 0, 21]])
        # self.arm.create_route("R_playful", [[2027, 981, 98, -11, 0, 72]])
        # self.arm.create_route("R_sleep", [[0, 1891, 1732, 48, 0, 0]])
        # self.arm.create_route("R_wakeup", [[0, 3523, 5032, 1, 0, 0]])
        # self.arm.create_route("R_leaving", [[-2689, 2612, 375, 27, 0, 18]])
        # self.arm.create_route("R_greet1", [[3665, 1774, 3013, 0, 0, 0]])
        # self.arm.create_route("R_curious", [[3664, 1774, 3013, 0, 0, 0]])

        routes = [r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, r13, r14]

        for r in routes:
            msg = "create_route:: " + r
            print "Sending message: ", msg
            self.arm_pub.publish(msg)
            time.sleep(1)

    def run(self):
        self.create()
        # time.sleep(3)
        # r = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     self.create()
        #     r.sleep()

if __name__ == '__main__':
    rc = RouteCreator()
    rc.run()
