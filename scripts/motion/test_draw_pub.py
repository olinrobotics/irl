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

    while not rospy.is_shutdown():
        time.sleep(1)
        msg = Edwin_Shape()
        msg.shape = "square"
        msg.x = 1
        msg.y = 2
        msg.z = 3
        pub.publish(msg)

if __name__ == '__main__':
    run()
