#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import String

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
        self.cmd = []

    def cmd_callback(self,data):
        self.cmd = [int(i) for i in data.split(" ")]
        self.is_building = True

    def model_callback(self,data):
        # parse the data to transform it to a np array
        # need to determine input format
        self.target_model = data

    def pickup(self):
        pass

    def coord_trans(self):
        pass

    def place_block(self):
        pass

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
