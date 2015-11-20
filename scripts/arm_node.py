#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String
import time

class ArmCommands:
    def __init__(self):
        rospy.init_node('robot_arm', anonymous=True)
        #self.arm.pub_arm = rospy.Publisher('/arm_debug', Twist, queue_size=10)
        rospy.Subscriber('/arm_cmd', String, self.arm_callback, queue_size=10)
        rospy.Subscriber('/behaviors_cmd', String, self.behavior_callback, queue_size=10)
        self.debug = False
        self.plan = []
        self.arm = st.StArm()
        self.arm.start()
        self.arm.calibrate()
        self.arm.home()
        self.behaviors = {}

        self.create_routes()
        self.create_behavoirs()

    def create_routes(self):
        self.arm.create_route("R_look", [[3664, 1774, 3013, 11, 0, 21]])
        self.arm.create_route("R_playful", [[2027, 981, 98, -11, 0, 72]])
        self.arm.create_route("R_sleep", [[0, 1891, 1732, 48, 0, 0]])
        self.arm.create_route("R_wakeup", [[0, 3523, 5032, 1, 0, 0]])
        self.arm.create_route("R_leaving", [[-2689, 2612, 375, 27, 0, 18]])

        self.routes = ["R_look", "R_playful", "R_sleep", "R_wakeup", "R_leaving"]

    def create_behavoirs(self):
        self.behaviors["butt_wiggle"] = "R_leaving, WA: 1000, WA: 800, WA: 1000"

    def run_routes(self):
        self.arm.continuous()
        for elem in self.routes:
            print elem
            self.arm.run_route(elem)
        print "test_done"

    def behavior_callback(self, cmdin):
        cmd = str(cmdin).replace("data: ", "")
        if cmd in self.behaviors.keys():
            cmd_list = self.behaviors[cmd].split(", ")
            for elem in cmd_list:
                if "R_" in elem:
                    self.arm.run_route(elem)
                else:
                    joint = elem.split(": ")[0]
                    pos = int(elem.split(": ")[1])
                    if joint == "H":
                        self.arm.rotate_hand(pos)
                    elif joint == "WR":
                        self.arm.rotate_wrist(pos)
                    elif joint == "E":
                        self.arm.rotate_elbow(pos)
                    elif joint == "S":
                        self.arm.rotate_shoulder(pos)
                    elif joint == "WA":
                        self.arm.rotate_waist(pos)

    def arm_callback(self, cmdin):
        self.arm.joint()
        cmd = str(cmdin).replace("data: ", "")
        if len(cmd.split(':: ')) > 1:
            param = cmd.split(':: ')[1]
            cmd = cmd.split(':: ')[0]
        print cmd
        if cmd == "de_energize":
            self.arm.de_energize()
        elif cmd == "run_routes":
            self.run_routes()
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
            print "setting speed to ", param
            self.arm.set_speed(float(param))
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
    object_tracker.run()
    rospy.spin()



