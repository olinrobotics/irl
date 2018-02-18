#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import String

class PathPlanner():
    def __init__(self):
        rospy.init_node("PathPlanner", anonymous=True)
        # receive model and xyz location
        self.model_sub = rospy.Subscriber("/model_cmd", String, self.model_callback,queue_size=1)
        self.coordinates_pub = rospy.Publisher("/coordinates_cmd", String, queue_size=10)
        self.joints_pub = rospy.Publisher("/joints_cmd", String, queue_size=10)
        self.grip_pub = rospy.Publisher("/grip_cmd", String, queue_size=10)

        self.num_row = 5
        self.num_col = 5
        self.max_height = 5

        self.curr_model = [[0 for col in range(5)] for row in range(5)]
        self.target_model = [[0 for col in range(5)] for row in range(5)]
        self.is_building = False
        self.build_seq = {}

    def model_callback(self,data):
        # parse the data to transform it to a np array
        # need to determine input format
        self.target_model = data

        self.is_building = True
        for i in range(self.num_row):
            for j in range(self.num_col):
                for layer in range(self.curr_model[i,j],self.target_model[i,j]):
                    if self.build_seq.get(layer) == None:
                        self.build_seq[layer] = [(i,j)]
                    else:
                        self.build_seq[layer].append((i,j))

    def pickup_block():

    def place_block(row,col,hei):
        

    def build_by_layer():
        for layer in range(self.max_height):
            if self.build_seq.get(layer) != None:
                for point in self.build_seq[layer]:
                    self.pickup_block()
                    print("Placing Block at " + str(point[0]) + ", " + str[point[1]] + ", " + str[layer])
                    self.place_block(point[0],point[1],layer)
        self.build_seq = {}
        self.curr_model = self.target_model

    def run(self):
        print("Path planner running")
        while not rospy.is_shutdown():
            try:
                # wait for user input
                if self.is_building:
                    self.build_by_layer()
                    time.sleep(1)
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    pp = PathPlanner();
