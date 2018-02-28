#!/usr/bin/env python

import rospy
import numpy as np
import time

from std_msgs.msg import String
from ur5_arm_node import Arm

class PathPlanner():
    def __init__(self):
        rospy.init_node("path_planner", anonymous=True)
        # receive model and xyz location
        self.model_pub = rospy.Publisher("/model_cmd", String, queue_size=1)
        # build_cmd is rececived from the brain as a string of three numbers as the xyz coordinates of the block to be placed
        self.cmd_sub = rospy.Subscriber("/build_cmd", String, self.cmd_callback,queue_size=1)
        self.coordinates_pub = rospy.Publisher("/coordinates_cmd", String, queue_size=10)
        self.joints_pub = rospy.Publisher("/joints_cmd", String, queue_size=10)
        self.grip_pub = rospy.Publisher("/grip_cmd", String, queue_size=10)

        self.curr_model = [[0 for col in range(5)] for row in range(5)]
        self.is_building = False

        self.my_arm = Arm(rospy)

        self.cmd = []

    def cmd_callback(self,data):
        self.cmd = [int(i) for i in data.split(" ")]
        self.is_building = True

    def pickup(self):
        # pick up the cube from a predefined location
        pass

    def coord_trans(self, base):
        # transform base coordinates to actual coordinate for the arm
        pass

    def place_block(self):
        # Go to universal starting position
        # Assume the block has already been picked up
        # pg_hover: 90, -90, 45, -45, -90, 0
        # coor : 110.29 -372.42 289.06
        # turn the wrist 90 degrees if other blocks are in the way
        if (self.cmd[1]>0 and self.curr_model[self.cmd[0],self.cmd[1]-1] >= self.cmd[2]) or (self.cmd[1]<4 and self.curr_model[self.cmd[0],self.cmd[1]+1] >= self.cmd[2]):
            msg = "pg_hover_alternate"
        else:
            msg = "pg_hover"
        print("Sending: ", msg)
        self.joints_pub.publish(msg)
        time.sleep(3)

        curr_location = self.my_arm.get_coordinates()
        cmd_location = self.coord_trans(self.cmd)

        # go to x,y coordinates
        msg = str(cmd_location[0]) + ' ' + str(cmd_location[1]) + ' ' + str(curr_location[2])
        print('Sending:' + msg)
        self.coordinates_pub.publish(msg)

        # go to z coordinate and place the block
        msg = str(cmd_location[0]) + ' ' + str(cmd_location[1]) + ' ' + str(cmd_location[2])
        print('Sending:' + msg)
        self.coordinates_pub.publish(msg)

        # Publish to gripper to release

        # update the current model2
        self.curr_model[self.cmd[0],self.cmd[1]] += 1
        self.model_pub.publish(str(self.curr_model))
        self.is_building = False

    def run(self):
        print("Path planner running")
        while not rospy.is_shutdown():
            try:
                # wait for user input
                if self.is_building:
                    self.pickup()
                    self.place_block()
                    time.sleep(1)
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    pp = PathPlanner()
    pp.run()
