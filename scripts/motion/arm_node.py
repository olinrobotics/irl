#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String, Int16
import time

class ArmCommands:
    def __init__(self):
        rospy.init_node('robot_arm', anonymous=True)

        rospy.Subscriber('/arm_cmd', String, self.arm_callback, queue_size=10)
        self.pub = rospy.Publisher('arm_debug', String, queue_size=10)
        self.pub2 = rospy.Publisher('arm_status', Int16, queue_size=10)

        self.debug = False
        self.plan = []
        self.arm = st.StArm()
        self.arm.start()
        print "ARM SPD IS: ", self.arm.get_speed()
        # self.arm.set_speed(10000)
        print "CALIBRATING"
        # self.arm.calibrate()
        print "HOMING"
        self.arm.home()
        self.behaviors = {}

        self.create_routes()
        self.arm.run_route("R_mv2")
        # self.arm.run_route("R_ttt")

    def create_routes(self):
        #Moves in units of thousands
        self.arm.create_route("R_mv2", [[3296, 2308, 999, 0, 0, 0], [200, 2400, 1800, 720, 240, 2.1]])
        self.arm.create_route("R_stare", [[3296, 2308, 999, 0, 0, 0]])
        self.arm.create_route("R_ttt", [[200, 2400, 1800, 720, 240, 2.1]])
        self.arm.create_route("R_look", [[3664, 1774, 3013, 11, 0, 21]])
        self.arm.create_route("R_playful", [[2027, 981, 98, -11, 0, 72]])
        self.arm.create_route("R_sleep", [[0, 1891, 1732, 48, 0, 0]])
        self.arm.create_route("R_wakeup", [[0, 3523, 5032, 1, 0, 0]])
        self.arm.create_route("R_leaving", [[-2689, 2612, 375, 27, 0, 18]])
        self.arm.create_route("R_greet1", [[3665, 1774, 3013, 0, 0, 0]])
        self.arm.create_route("R_curious", [[3664, 1774, 3013, 0, 0, 0]])

        self.routes = ["R_mv2", "R_stare", "R_ttt", "R_look", "R_playful", "R_sleep", "R_wakeup", "R_leaving, R_greet1", "R_curious"]

    def arm_callback(self, cmdin):
        self.arm.joint()
        cmd = str(cmdin).replace("data: ", "")
        if len(cmd.split(':: ')) > 1:
            param = cmd.split(':: ')[1]
            cmd = cmd.split(':: ')[0]
        print cmd
        if cmd == "de_energize":
            self.arm.de_energize()
        elif cmd == "R_ttt":
            print "MOVING TO TTT HOME POSITION"
            self.arm.run_route("R_ttt")
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
            #SPD is also in units of 1000
            print "setting speed to ", param
            self.arm.set_speed(float(param))
            print "ARM SPD IS: ", self.arm.get_speed()
        elif cmd == "set_point":
            self.arm.set_point(param)
        elif cmd == "get_accel":
            accel = self.arm.get_accel()
        elif cmd == "set_accel":
            self.arm.set_accel(param)
        elif cmd == "run_route":
            self.pub2.publish(1)
            self.arm.run_route(param)
            self.pub2.publish(0)
        elif cmd == "move_to":
            #NOTE: move_to is in units of mm
            temp = param.split(", ")
            x = temp[0]
            y = temp[1]
            z = temp[2]
            pitch = temp[3]
            self.pub2.publish(1)
            self.arm.move_to(x,y,z,self.arm.debug)
            self.pub2.publish(0)
        elif cmd == "rotate_wrist":
            self.pub2.publish(1)
            self.arm.rotate_wrist(param)
            self.pub2.publish(0)
        elif cmd == "rotate_wrist_rel":
            self.pub2.publish(1)
            self.arm.rotate_wrist_rel(param)
            self.pub2.publish(0)
        elif cmd == "rotate_hand":
            self.pub2.publish(1)
            self.arm.rotate_hand(param)
            self.pub2.publish(0)
        elif cmd == "rotate_elbow":
            self.pub2.publish(1)
            self.arm.rotate_elbow(param)
            self.pub2.publish(0)
        elif cmd == "rotate_shoulder":
            self.pub2.publish(1)
            self.arm.rotate_shoulder(param)
            self.pub2.publish(0)
        elif cmd == "rotate_waist":
            self.pub2.publish(1)
            self.pub2.publish(0)
            self.arm.rotate_waist(param)
        elif cmd == "rotate_hand_rel":
            self.pub2.publish(1)
            self.arm.rotate_hand_rel(param)
            self.pub2.publish(0)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    arm_eng = ArmCommands()
    arm_eng.run()
