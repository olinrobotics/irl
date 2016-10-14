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

        rospy.Subscriber('/arm_cmd', String, self.arm_callback, queue_size=1)
        self.debug_pub = rospy.Publisher('arm_debug', String, queue_size=10)
        self.status_pub = rospy.Publisher('arm_status', Int16, queue_size=10)

        self.debug = False
        self.plan = []
        self.arm = st.StArm()
        print "CALIBRATING"
        self.arm.initial_calibration()
        self.arm.start()

        print "ARM SPD IS: ", self.arm.get_speed()
        print "ARM ACCEL IS: ", self.arm.get_accel()

        self.arm.set_speed(10000)
        print "HOMING"
        self.arm.home()

        self.debug_pub.publish("HOMING DONE")

    def arm_callback(self, cmdin):
        self.arm.joint()
        cmd = cmdin.data
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
            print "CREATING NEW ROUTE"
            param = param.split("; ")
            route_name = param[0]

            commands = []
            numbers = param[1].split(", ")

            if len(numbers)%6 != 0:
                print "INVALID ROUTE"
                return

            i = 0
            while i < len(numbers):
                route = []
                j = 0
                while j < 6:
                    route.append(int(numbers[i+j]))
                    j += 1
                commands.append(route)
                i += j

            print "CREATING ROUTE: ", route_name
            print "CMDS: ", commands
            self.arm.create_route(route_name, commands)

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
            #ACCEL is also in units of 1000
            print "setting accel to ", param
            self.arm.set_accel(float(param))
        elif cmd == "run_route":
            self.status_pub.publish(1)
            res = self.arm.run_route(param)

            if not res:
                self.debug_pub.publish("ROUTE NOT FOUND: " + param)
            self.status_pub.publish(0)
        elif cmd == "move_to":
            #NOTE: move_to is in units of mm
            temp = param.split(", ")
            x = temp[0]
            y = temp[1]
            z = temp[2]
            pitch = temp[3]
            self.status_pub.publish(1)
            self.arm.move_to(x,y,z,self.arm.debug)
            self.status_pub.publish(0)
        elif cmd == "rotate_wrist":
            self.status_pub.publish(1)
            self.arm.rotate_wrist(param)
            self.status_pub.publish(0)
        elif cmd == "rotate_wrist_rel":
            self.status_pub.publish(1)
            self.arm.rotate_wrist_rel(param)
            self.status_pub.publish(0)
        elif cmd == "rotate_hand":
            self.status_pub.publish(1)
            self.arm.rotate_hand(param)
            self.status_pub.publish(0)
        elif cmd == "rotate_elbow":
            self.status_pub.publish(1)
            self.arm.rotate_elbow(param)
            self.status_pub.publish(0)
        elif cmd == "rotate_shoulder":
            self.status_pub.publish(1)
            self.arm.rotate_shoulder(param)
            self.status_pub.publish(0)
        elif cmd == "rotate_waist":
            self.status_pub.publish(1)
            self.arm.rotate_waist(param)
            self.status_pub.publish(0)
        elif cmd == "rotate_waist_rel":
            self.status_pub.publish(1)
            print "RELATIVE WA ROTATION"
            self.arm.rotate_waist(param)
            self.status_pub.publish(0)
        elif cmd == "rotate_hand_rel":
            self.status_pub.publish(1)
            self.arm.rotate_hand_rel(param)
            self.status_pub.publish(0)
        elif cmd == "sleeping":
            time.sleep(float(param))

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    arm_eng = ArmCommands()
    arm_eng.run()
