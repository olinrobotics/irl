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

        self.debug = False

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
            location = self.arm.where()
            print location
        elif cmd == "create_route":
            #TODO: Implement this
            route_name = "TEST"
            commands = "NOTHING"
            self.create_route(route_name, commands, self.debug)
        elif cmd == "calibrate":
            self.calibrate()
        elif cmd == "home":
            self.home()
        elif cmd == "get_speed":
            speed = self.get_speed()
        elif cmd == "set_speed":
            speed = 1
            self.set_speed(speed)
        elif cmd == "set_point":
            name = "TEST"
            self.set_point(name)
        elif cmd == "get_accel":
            accel = self.get_accel()
        elif cmd == "set_accel":
            accel = 0.1
            self.set_accel(accel)
        elif cmd == "run_route":
            route = "TEST"
            self.run_route(route)
        elif cmd == "move_to":
            x = 0
            y = 0
            z = 0
            self.move_to(x,y,z,self.debug)
        elif cmd == "rotate_wrist":
            roll = 0
            self.rotate_wrist(roll)
        elif cmd == "rotate_wrist_rel":
            roll_inc = 0
            self.rotate_wrist_rel(roll_inc)
        elif cmd == "rotate_hand":
            pitch = 0
            self.rotate_hand(pitch)
        elif cmd == "rotate_elbow":
            pitch = 0
            self.rotate_elbow(pitch)
        elif cmd == "rotate_shoulder":
            pitch = 0
            self.rotate_shoulder(pitch)
        elif cmd == "rotate_waist":
            pitch = 0
            self.rotate_waist(pitch)
        elif cmd == "rotate_hand_rel":
            pitch_inc = 0
            self.rotate_hand_rel(pitch_inc)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    object_tracker = ArmCommands()
    # object_tracker.run_arm()
    object_tracker.run()
    rospy.spin()