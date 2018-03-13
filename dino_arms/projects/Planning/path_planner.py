#!/usr/bin/env python

import rospy
import numpy as np
import time
import sys
import math
sys.path.append('/home/yichen/catkin_ws/src/irl/dino_arms/projects/Controls')

from std_msgs.msg import String
from ur5_arm_node import Arm

# python path_planner.py _robot_ip:=10.42.0.54
class PathPlanner():
    def __init__(self):
        rospy.init_node("path_planner", anonymous=True)
        # receive model and xyz location
        self.model_pub = rospy.Publisher("/model_cmd", String, queue_size=1)
        # build_cmd is rececived from the brain as a string of three numbers as the xyz coordinates of the block to be placed
        self.cmd_sub = rospy.Subscriber("/build_cmd", String, self.cmd_callback,queue_size=1)
        self.coordinates_pub = rospy.Publisher("/coordinates_cmd", String, queue_size=10)
        self.joints_pub = rospy.Publisher("/behaviors_cmd", String, queue_size=10)
        # Make query about the joints/coordinates information and wait for callback
        self.query_pub = rospy.Publisher("/query_cmd", String, queue_size=10)
        self.info_sub = rospy.Subscriber("/arm_info", String, self.info_callback, queue_size=10)

        self.grip_pub = rospy.Publisher("/grip_cmd", String, queue_size=10)

        self.curr_model = [[0 for col in range(5)] for row in range(5)]
        self.is_building = False

        self.cmd = []
        self.query = ""
        self.curr_location = []
        self.curr_angle = []

    def cmd_callback(self,data):
        # cmd is from 0 to 4
        print(data.data.split(" "))
        self.cmd = [int(math.trunc(float(i))) for i in data.data.split(" ")]
        print(self.cmd)
        self.is_building = True

    def info_callback(self,data):
        if self.query == "coordinates":
            arm_info = data.data[1:len(data.data)-1]
            print("Arm coordinates are: " + arm_info)
            self.curr_location = [float(i) for i in arm_info.split(',')]
        elif self.query == "joints":
            arm_info = data.data[1:len(data.data)-1]
            print("Joing angles are: " + arm_info)
            self.curr_angle = [float(i) for i in arm_info.split(',')]

    def pickup(self):
        # TODO pick up the cube from a predefined location
        pass

    def coord_trans(self, base):
        # transform base coordinates to actual coordinate for the arm
        # Each cube has the dimension of 101.6mm. Assume the default location is 110.29, -372.42, 289.06 for now
        # zero z is -191.0
        unit_length = 0.1016
        default = [0.1103, -0.3718, 0.2890]
        default[1] = default[1] + 2 * unit_length
        # TODO redefine zero of z

        real_x = default[0] - (base[0]-2) * unit_length
        real_y = default[1] - base[1] * unit_length
        real_z = default[2] + base[2] * unit_length
        return [real_x, real_y, real_z]

    def place_block(self):
        # Go to universal starting position
        # Assume the block has already been picked up
        # pg_hover: 90, -90, 45, -45, -90, 0
        # coor : 110.29 -372.42 289.06
        # turn the wrist 90 degrees if other blocks are in the way

        if (self.cmd[1]>0 and self.curr_model[self.cmd[0]][self.cmd[1]-1] >= self.cmd[2]) or (self.cmd[1]<4 and self.curr_model[self.cmd[0]][self.cmd[1]+1] >= self.cmd[2]):
            msg = "pg_hover_alternate"
        else:
            msg = "pg_hover"
        print("Sending: ", msg)
        self.joints_pub.publish(msg)
        time.sleep(3)

        # make query to ur5_arm_node and wait for callback
        self.query = "coordinates"
        self.query_pub.publish(self.query)
        time.sleep(2)

        print("cmd is: " + str(self.cmd))
        cmd_location = self.coord_trans(self.cmd)

        # go to x,y coordinates
        msg = str(cmd_location[0]) + ' ' + str(cmd_location[1]) + ' ' + str(self.curr_location[2])
        print('Sending:' + msg)
        self.coordinates_pub.publish(msg)
        time.sleep(3)

        # go to z coordinate and place the block
        msg = str(cmd_location[0]) + ' ' + str(cmd_location[1]) + ' ' + str(cmd_location[2])
        print('Sending:' + msg)
        self.coordinates_pub.publish(msg)
        time.sleep(3)

        # TODO Publish to gripper to release

        # update the current model2
        self.curr_model[self.cmd[0]][self.cmd[1]] += 1
        self.model_pub.publish(str(self.curr_model))

    def run(self):
        print("Path planner running")
        while not rospy.is_shutdown():
            try:
                # wait for user input
                if self.is_building:
                    # self.pickup()
                    if self.cmd[0]>=0 and self.cmd[0]<=4 and self.cmd[1]>=0 and self.cmd[1]<=4:
                        self.place_block()
                    else:
                        print('Input is illegal')
                    self.is_building = False
                    time.sleep(1)
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    pp = PathPlanner()
    pp.run()
