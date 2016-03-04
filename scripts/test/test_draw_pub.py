#!/usr/bin/env python
import rospy
import math
# import st
import numpy as np
from std_msgs.msg import String
from edwin.msg import *
import time

def run():
    rospy.init_node('arm_tester', anonymous=True)
    pub = rospy.Publisher('draw_cmd', Edwin_Shape, queue_size=10)
    arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
    time.sleep(1)

    while not rospy.is_shutdown():
        msg = Edwin_Shape()
        msg.shape = "line"
        msg.x = 0
        msg.y = 4000
        #note that Z should be a function of y.
        msg.z = int(-770 - ((msg.y - 2500)/10))
        pub.publish(msg)
        time.sleep(30)

        # # #Board X and Y positions
        # b_x = 0
        # b_y = 4000
        # b_w = 250

        # #Sector centroids
        # b_centers = {}
        # b_centers[0] = (b_x - 2.25*b_w, self.b_y + 1.5*self.b_w)
        # b_centers[1] = (b_x, b_y + 2*self.b_w)
        # b_centers[2] = (b_x + 2*b_w, self.b_y + 1.5*self.b_w)

        # b_centers[3] = (b_x - 2*b_w, self.b_y)
        # b_centers[4] = (b_x, b_y)
        # b_centers[5] = (b_x + 2*self.b_w, self.b_y)

        # b_centers[6] = (b_x - 2*self.b_w, self.b_y - 1.5*self.b_w)
        # b_centers[7] = (b_x, selfb_y - 1.5*self.b_w)
        # b_centers[8] = (b_x + 2*b_w, self.b_y - 1.5*self.b_w)



        # msg.shape = "circle"
        # msg.x = 0
        # msg.y = 4000
        # #note that Z should be a function of y.
        # msg.z = int(-760 - ((msg.y - 2500)/10))
        # pub.publish(msg)
        # time.sleep(30)

        # msg.shape = "square"
        # msg.x = 0
        # msg.y = 4000
        # #note that Z should be a function of y.
        # msg.z = int(-760 - ((msg.y - 2500)/10))
        # pub.publish(msg)
        # time.sleep(30)



        # motions = ["data: move_to:: 100, 2200, 500, 0", "sending:  data: rotate_wrist:: -100", "sending:  data: rotate_hand:: 50"]
        # for motion in motions:
        #     print "pub: ", motion
        #     arm_pub.publish(motion)
        #     time.sleep(2)

if __name__ == '__main__':
    run()
