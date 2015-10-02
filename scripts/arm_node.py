#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String

class ArmCommands:
    def __init__(self):
        rospy.init_node('robot_arm', anonymous=True)
        #self.pub_arm = rospy.Publisher('/arm_debug', Twist, queue_size=10)
        rospy.Subscriber('/arm_cmd', String, self.callback, queue_size=10)

        self.plan = []
        self.arm = st.StArm()
        self.arm.start()
        #self.arm.calibrate()
        self.arm.home()

    def run_arm(self):
        self.arm.continuous()
        self.arm.create_route("TEST1",[[-2992, 0, 5500], [-2000,1800,5500]])
        self.arm.run_route("TEST1")
        print "test_done"

    def callback(self, cmdin):
        cmd = str(cmdin).replace("data: ", "")
        print cmd
        if cmd == "de_energize":
            self.arm.de_energize()
        elif cmd == "energize":
            self.arm.energize()
        elif cmd == "where":
            self.arm.where()

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    object_tracker = ArmCommands()
    # object_tracker.run_arm()
    object_tracker.run()
    rospy.spin()