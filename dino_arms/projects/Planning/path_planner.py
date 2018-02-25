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
        self.model_sub = rospy.Subscriber("/model_cmd", String, self.model_callback,queue_size=1)
        self.model_pub = rospy.Subscriber("/model_cmd", String, queue_size=1)
        self.cmd_pub = rospy.Subscriber("/build_cmd", String, self.cmd_callback,queue_size=1)
        self.coordinates_pub = rospy.Publisher("/coordinates_cmd", String, queue_size=10)
        self.joints_pub = rospy.Publisher("/joints_cmd", String, queue_size=10)
        self.grip_pub = rospy.Publisher("/grip_cmd", String, queue_size=10)

        self.curr_model = [[0 for col in range(5)] for row in range(5)]
        self.target_model = [[0 for col in range(5)] for row in range(5)]
        self.is_building = False

        self.my_x = -1
        self.my_y = -1
        self.my_z = -1

        self.my_arm = Arm(rospy)

        self.cmd = []

    def cmd_callback(self,data):
        self.cmd = [int(i) for i in data.split(" ")]
        self.is_building = True

    def model_callback(self,data):
        # parse the data to transform it to a np array
        # need to determine input format
        self.target_model = data

    def coord_trans(self):
        pass

    def place_block(self):
        # Go to universal starting position
        # Assume the block has already been picked up
        # pg_hover: 90, -90, 45, -45, -90, 0
        msg = "tp_camera"
        print("Sending: ", msg)
        self.joints_pub.publish(msg)
        time.sleep(3)

        # go to x,y coordinates

        # go to z coordinate and place the block

    def run(self):
        print("Path planner running")
        while not rospy.is_shutdown():
            try:
                # wait for user input
                if self.is_building:
                    self.pickup();
                    self.place_block();
                    time.sleep(1)
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    pp = PathPlanner();
