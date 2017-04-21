#!/usr/bin/env python

import sys
import rospy
from edwin.srv import *
import time

class SrvTest(object):

    def __init__(self):
        self.status = -1
        self.behavior_pub = rospy.Publisher('/behaviors_cmd', String, queue_size=10)
        rospy.Subscriber('/arm_status', String, self.callback, queue_size=10)

    def callback(self, data):
        if data. == "busy":
            self.status = 0
        elif data == "done":
            self.status = 1


    def run(self):

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.behavior_pub.publish('heart')
            time.sleep(1)
            while self.status == 0:
                print "waiting on look to finish"
            self.behavior_pub.publish('bow')
            time.sleep(1)
            while self.status == 0:
                print "waiting on bow to finish"
            r.sleep()

# def add_two_ints_client(cmd):
#     rospy.wait_for_service('arm_cmd')
#     cmd_fnc = rospy.ServiceProxy('arm_cmd', arm_cmd)
#     print "i have requested"
#     try:
#         resp1 = cmd_fnc(cmd)
#         print "request finished"
#
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e

if __name__ == "__main__":
    node = SrvTest()
    node.run()
