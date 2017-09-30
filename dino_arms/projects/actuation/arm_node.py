#!/usr/bin/env python
from __future__ import absolute_import
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String, Int16, Header
from sensor_msgs.msg import JointState
import time
from irl.srv import arm_cmd

class ArmCommands(object):
    def __init__(self):
        rospy.init_node(u'robot_arm', anonymous=True)
        s = rospy.Service(u'arm_cmd', arm_cmd, self.arm_callback)
        print u"Service ONLINE"

        # rospy.Subscriber('/arm_cmd', String, self.arm_callback, queue_size=1)
        self.debug_pub = rospy.Publisher(u'arm_debug', String, queue_size=10)
        # self.status_pub = rospy.Publisher('arm_status', String, queue_size=10)

        self.joint_state_pub = rospy.Publisher(u'joint_states', JointState, queue_size=10)
        self.joint_state_msg = JointState()

        self.debug = False
        self.plan = []
        self.arm = st.StArm()
        print u"CALIBRATING"
        self.arm.initial_calibration()
        self.arm.start()

        print u"ARM SPD IS: ", self.arm.get_speed()
        print u"ARM ACCEL IS: ", self.arm.get_accel()

        self.arm.set_speed(10000)
        self.arm.home()
        print u"HOMING"

        self.debug_pub.publish(u"HOMING DONE")

    def arm_callback(self, cmdin):
        self.arm.joint()
        cmd = cmdin.cmd
        cmd = unicode(cmd).replace(u"data: ", u"")
        if len(cmd.split(u':: ')) > 1:
            param = cmd.split(u':: ')[1]
            cmd = cmd.split(u':: ')[0]
        print cmd
        if cmd == u"de_energize":
            self.arm.de_energize()
        elif cmd == u"energize":
            self.arm.energize()
        elif cmd == u"where":
            location = self.arm.where()

            #print 'location', location

        elif cmd == u"create_route":
            print u"CREATING NEW ROUTE"
            param = param.split(u"; ")
            route_name = param[0]

            commands = []
            numbers = param[1].split(u", ")

            if len(numbers)%6 != 0:
                print u"INVALID ROUTE"
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

            print u"CREATING ROUTE: ", route_name
            print u"CMDS: ", commands
            self.arm.create_route(route_name, commands)

        elif cmd == u"calibrate":
            self.arm.calibrate()
        elif cmd == u"home":
            self.arm.home()
        elif cmd == u"get_speed":
            speed = self.arm.get_speed()
        elif cmd == u"set_speed":
            #SPD is also in units of 1000
            print u"setting speed to ", param
            self.arm.set_speed(float(param))
            print u"ARM SPD IS: ", self.arm.get_speed()
        elif cmd == u"set_point":
            self.arm.set_point(param)
        elif cmd == u"get_accel":
            accel = self.arm.get_accel()
        elif cmd == u"set_accel":
            #ACCEL is also in units of 1000
            print u"setting accel to ", param
            self.arm.set_accel(float(param))
        elif cmd == u"run_route":
            # self.status_pub.publish(1)
            res = self.arm.run_route(param)

            if not res:
                self.debug_pub.publish(u"ROUTE NOT FOUND: " + param)
            # self.status_pub.publish(0)
        elif cmd == u"move_to":
            #NOTE: move_to is in units of mm
            temp = param.split(u", ")
            x = temp[0]
            y = temp[1]
            z = temp[2]
            pitch = temp[3]
            # self.status_pub.publish(1)
            self.arm.move_to(x,y,z,self.arm.debug)
            # self.status_pub.publish(0)
        elif cmd == u"rotate_wrist":
            # self.status_pub.publish(1)
            self.arm.rotate_wrist(param)
            # self.status_pub.publish(0)
        elif cmd == u"rotate_wrist_rel":
            # self.status_pub.publish(1)
            self.arm.rotate_wrist_rel(param)
            # self.status_pub.publish(0)
        elif cmd == u"rotate_hand":
            # self.status_pub.publish(1)
            self.arm.rotate_hand(param)
            # self.status_pub.publish(0)
        elif cmd == u"rotate_elbow":
            # self.status_pub.publish(1)
            self.arm.rotate_elbow(param)
            # self.status_pub.publish(0)
        elif cmd == u"rotate_shoulder":
            # self.status_pub.publish(1)
            self.arm.rotate_shoulder(param)
            # self.status_pub.publish(0)
        elif cmd == u"rotate_waist":
            # self.status_pub.publish(1)
            self.arm.rotate_waist(param)
            # self.status_pub.publish(0)
        elif cmd == u"rotate_waist_rel":
            # self.status_pub.publish(1)
            print u"RELATIVE WA ROTATION"
            self.arm.rotate_waist(param)
            # self.status_pub.publish(0)
        elif cmd == u"rotate_hand_rel":
            # self.status_pub.publish(1)
            self.arm.rotate_hand_rel(param)
            # self.status_pub.publish(0)
        elif cmd == u"sleeping":
            time.sleep(float(param))
        elif cmd == u"get_joint_states":
            self.status_pub.publish(1)
            joint_states = self.arm.get_joint_states()

            h = Header()
            h.frame_id = u"base_link"
            h.stamp = rospy.Time.now()

            self.joint_state_msg.header = h
            self.joint_state_msg.name = [u'joint_1',u'joint_2',u'joint_3',u'joint_4',u'joint_5']
            # TODO : fix these names ... here for compatibility with URDF model
            self.joint_state_msg.position = joint_states
            self.joint_state_msg.velocity = [0 for _ in xrange(5)]
            self.joint_state_msg.effort = [0 for _ in xrange(5)]

            self.joint_state_pub.publish(self.joint_state_msg)

            self.status_pub.publish(0)
        elif cmd == u"test":
            self.status_pub.publish(1)
            self.arm.test()
            self.status_pub.publish(0)



        return [u"I have completed the command"]


    def run(self):
        print u"Service is ready to go"
        rospy.spin()
        # r = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     r.sleep()

if __name__ == u"__main__":
    arm_eng = ArmCommands()
    arm_eng.run()
