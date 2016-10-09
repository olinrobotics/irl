#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String, Int16
from edwin.msg import *
import time


class Waver:
    def __init__(self):
        rospy.init_node('edwin_presence', anonymous = True)
        rospy.Subscriber('single_servo', Int16, self.wave_callback, queue_size=10)

        self.behavior_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
        self.arm_pub.publish("data: set_speed:: 3000")


    def wave_callback(self, data):

        time.sleep(1)
        print "I received a piece of data which is ", data

        if data == 1:
            print "I saw you wave! Hello!"
            msg = "data: R_nudge"
            self.behavior_pub.publish(msg)


    def run(self):
        print "running"
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    wave = Waver()
    time.sleep(2)
    wave.run()
