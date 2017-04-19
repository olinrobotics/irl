#!/usr/bin/env python

import sys
import rospy
from edwin.srv import *

def add_two_ints_client(cmd):
    rospy.wait_for_service('arm_cmd')
    try:
        cmd_fnc = rospy.ServiceProxy('arm_cmd', AddTwoInts)
        resp1 = cmd_fnc(cmd)
        return resp1.response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    index = 0
    while True:
        index += 1
        print index
        print add_two_ints_client(str("data: rotate_waist:: " + 0))
        print add_two_ints_client(str("data: rotate_waist:: " + 5000))
        print " "
