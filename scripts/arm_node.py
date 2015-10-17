#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String

class ArmCommands:
    def __init__(self):
        rospy.init_node('robot_arm', anonymous=True)
        #self.arm.pub_arm = rospy.Publisher('/arm_debug', Twist, queue_size=10)
        rospy.Subscriber('/arm_cmd', String, self.callback, queue_size=10)

        self.debug = False

        self.plan = []
        self.arm = st.StArm()
        self.arm.start()
        #self.arm.arm.calibrate()
        self.arm.home()
        #self.run_arm()
        #self.arm.move_to(-2992, 0, 5500)
        #self.arm.create_route('hello', [[1,2,3],[4,5,6],[7,8,9]])

    def run_arm(self):
        self.arm.purge()
        self.arm.move_to(-1000, 0, 5500)
        self.arm.move_to(-2992, 50, 4000)
        self.arm.move_to(-2000, 1800, 5500)
        self.arm.continuous()
        self.arm.create_route("TEST1",[[-1000, 0, 5500], [-2992, 50, 4000], [-2000, 1800, 5500]])
        self.arm.run_route("TEST1")
        print "test_done"

    def callback(self, cmdin):
        self.arm.joint()
        cmd = str(cmdin).replace("data: ", "")
        if len(cmd.split(':: ')) > 1:
            param = cmd.split(':: ')[1]
            cmd = cmd.split(':: ')[0]
        print cmd
        if cmd == "de_energize":
            self.arm.de_energize()
        elif cmd == "energize":
            self.arm.energize()
        elif cmd == "where":
            location = self.arm.where()
            print location
        elif cmd == "create_route":
            #TODO: Implement this
            route_name = "TEST"
            commands = "NOTHING"
            self.arm.create_route(route_name, commands, self.arm.debug)
        elif cmd == "calibrate":
            self.arm.calibrate()
        elif cmd == "home":
            self.arm.home()
        elif cmd == "get_speed":
            speed = self.arm.get_speed()
        elif cmd == "set_speed":
            self.arm.set_speed(param)
        elif cmd == "set_point":
            self.arm.set_point(param)
        elif cmd == "get_accel":
            accel = self.arm.get_accel()
        elif cmd == "set_accel":
            self.arm.set_accel(param)
        elif cmd == "run_route":
            self.arm.run_route(param)
        elif cmd == "move_to":
            temp = param.split(", ")
            x = temp[0]
            y = temp[1]
            z = temp[2]
            self.arm.move_to(x,y,z,self.arm.debug)
        elif cmd == "rotate_wrist":
            self.arm.rotate_wrist(param)
        elif cmd == "rotate_wrist_rel":
            self.arm.rotate_wrist_rel(param)
        elif cmd == "rotate_hand":
            self.arm.rotate_hand(param)
        elif cmd == "rotate_elbow":
            self.arm.rotate_elbow(param)
        elif cmd == "rotate_shoulder":
            self.arm.rotate_shoulder(param)
        elif cmd == "rotate_waist":
            self.arm.rotate_waist(param)
        elif cmd == "rotate_hand_rel":
            self.arm.rotate_hand_rel(param)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    object_tracker = ArmCommands()
    object_tracker.run_arm()

    object_tracker.run()
    rospy.spin()



