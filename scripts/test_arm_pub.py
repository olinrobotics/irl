#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String

rospy.init_node('arm_tester', anonymous=True)
pub = rospy.Publisher('arm_cmd', String, queue_size=10)

while not rospy.is_shutdown():
    msg = str(raw_input("Your command: "))
    pub.publish(msg)





