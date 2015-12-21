#!/usr/bin/env python
import rospy
import math
import st
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
        # msg = Edwin_Shape()
        # msg.shape = "board"
        # msg.x = 0
        # msg.y = 4000
        # #note that Z should be a function of y.
        # msg.z = int(-650 - ((msg.y - 2500)/8))
        # pub.publish(msg)
        # time.sleep(15)

        motions = ["data: move_to:: 100, 2200, 500, 0", "sending:  data: rotate_wrist:: -100", "sending:  data: rotate_hand:: 50"]
        for motion in motions:
            print "pub: ", motion
            arm_pub.publish(motion)
            time.sleep(2)

if __name__ == '__main__':
    run()
