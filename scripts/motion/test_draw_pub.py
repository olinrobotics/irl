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
    time.sleep(1)

    while not rospy.is_shutdown():
        centers = [(-1500,5000), (0,5000), (1500,5000), (-1500,4000), (0,4000), (1500,4000), (-1500,3000), (0,3000), (1500,3000)]
        for center in centers:
            msg = Edwin_Shape()
            msg.shape = "board"
            msg.x = center[0]
            msg.y = center[1]
            #note that Z should be a function of y.
            msg.z = int(-650 - ((msg.y - 2500)/8))
            pub.publish(msg)
            time.sleep(15)


if __name__ == '__main__':
    run()
