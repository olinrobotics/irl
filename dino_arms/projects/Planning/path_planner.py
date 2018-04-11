#!/usr/bin/env python

import rospy
import numpy as np
import time
import sys
import math
sys.path.append('/home/yichen/catkin_ws/src/irl/dino_arms/projects/Controls')

from std_msgs.msg import String
from ur5_arm_node import Arm
from irl.msg import Cube_Structures, Cube, Structure

# python path_planner.py _robot_ip:=10.42.0.54
class PathPlanner():
    '''
    This is the path planner for the Spring 18 Interactive Robotics Lab project
    This module is capable of placing the blocks at the designated location
    without collision with other blocks. The program
    '''
    def __init__(self):
        rospy.init_node("path_planner", anonymous=True)
        # receive model and xyz location
        self.model_pub = rospy.Publisher("/model_cmd", String, queue_size=1)
        # build_cmd is rececived from the brain as a string of three numbers as the xyz coordinates of the block to be placed
        self.cmd_sub = rospy.Subscriber("/build_cmd", Cube_Structures, self.cmd_callback,queue_size=1)
        self.coordinates_pub = rospy.Publisher("/coordinates_cmd", String, queue_size=10)
        self.joints_pub = rospy.Publisher("/behaviors_cmd", String, queue_size=10)
        # Make query about the joints/coordinates information and wait for callback
        self.query_pub = rospy.Publisher("/query_cmd", String, queue_size=10)
        self.info_sub = rospy.Subscriber("/arm_info", String, self.info_callback, queue_size=10)

        self.grip_pub = rospy.Publisher("/grip_cmd", String, queue_size=10)

        self.curr_model = [[0 for col in range(5)] for row in range(5)]
        self.is_building = False

        # geometry of the cube
        self.unit_length = 0.1016

        self.grid_building = Structure()
        self.real_building = Structure()
        self.query = ""
        self.curr_location = []
        self.curr_angle = []
        self.push_flag = 0 # 0:don't need pushing; 1:push from back; 2:push from front; 3:push from left; 4: push from right
        self.push_instruction = [(0,0), (0, -1.0/3), (0,1.0/3), (-1.0/3, 0), (1.0/3, 0)]

    def cmd_callback(self,data):
        '''
        Parse the build command from the brain
        '''
        # cmd is from 0 to 4
        self.grid_building = data.grid_building
        self.real_building = data.real_building
        self.is_building = True

    def info_callback(self,data):
        '''
        Parse realtime coordinates / joint angles of the arm
        '''
        if self.query == "coordinates":
            arm_info = data.data[1:len(data.data)-1]
            # print("Arm coordinates are: " + arm_info)
            self.curr_location = [float(i) for i in arm_info.split(',')]
        elif self.query == "joints":
            arm_info = data.data[1:len(data.data)-1]
            # print("Joing angles are: " + arm_info)
            self.curr_angle = [float(i) for i in arm_info.split(',')]

    def pickup(self):
        # TODO pick up the cube from a predefined location
        pass

    def coord_trans(self, base):
        '''
        transform base coordinates to actual coordinate for the arm to place the block
        Each cube has the dimension of 101.6mm. Assume the default location is 110.29, -372.42, 289.06 for now
        zero z is -191.0
        Not being used anymore
        '''
        default = [0.1103, -0.3718, 0.2890]
        default[1] = default[1] + 2 * self.unit_length
        # TODO redefine zero of z

        real_x = default[0] - (base[0]-2) * self.unit_length + self.push_instruction[self.push_flag][0] * self.unit_length
        real_y = default[1] - base[1] * self.unit_length + self.push_instruction[self.push_flag][1] * self.unit_length
        real_z = default[2] + base[2] * self.unit_length
        return [real_x, real_y, real_z]

    def push_block(self):
        '''
        Model that deal with block pushing
        '''
        # make query to ur5 arm for current coordinates
        self.query = "coordinates"
        self.query_pub.publish(self.query)
        time.sleep(2)

        # go up 2 units
        self.curr_location[2] = self.curr_location[2] + 2 * self.unit_length;
        msg = str(self.curr_location[0]) + ' ' + str(self.curr_location[1]) + ' ' + str(self.curr_location[2])
        print('Sending:' + msg)
        self.coordinates_pub.publish(msg)
        time.sleep(2)

        # go outward
        self.curr_location[0] = self.curr_location[0] + self.push_instruction[self.push_flag][0] * 4 * self.unit_length;
        self.curr_location[1] = self.curr_location[1] + self.push_instruction[self.push_flag][1] * 4 * self.unit_length;
        msg = str(self.curr_location[0]) + ' ' + str(self.curr_location[1]) + ' ' + str(self.curr_location[2])
        print('Sending:' + msg)
        self.coordinates_pub.publish(msg)
        time.sleep(2)

        # go down 2 units
        self.curr_location[2] = self.curr_location[2] - 2 * self.unit_length;
        msg = str(self.curr_location[0]) + ' ' + str(self.curr_location[1]) + ' ' + str(self.curr_location[2])
        print('Sending:' + msg)
        self.coordinates_pub.publish(msg)
        time.sleep(2)

        # push inward
        self.curr_location[0] = self.curr_location[0] - self.push_instruction[self.push_flag][0] * 2 * self.unit_length;
        self.curr_location[1] = self.curr_location[1] - self.push_instruction[self.push_flag][1] * 2 * self.unit_length;
        msg = str(self.curr_location[0]) + ' ' + str(self.curr_location[1]) + ' ' + str(self.curr_location[2])
        print('Sending:' + msg)
        self.coordinates_pub.publish(msg)
        time.sleep(2)

    def place_block(self, grid_coord, real_coord):
        # Go to universal starting position
        # Assume the block has already been picked up
        # pg_hover: 90, -90, 45, -45, -90, 0
        # coor : 110.29 -372.42 289.06
        # turn the wrist 90 degrees if other blocks are in the way

        self.back_blocked = (grid_coord.y<4 and self.curr_model[grid_coord.x][grid_coord.y+1] > grid_coord.z)
        self.front_blocked = (grid_coord.y>0 and self.curr_model[grid_coord.x][grid_coord.y-1] > grid_coord.z)
        self.right_blocked = (grid_coord.x>0 and self.curr_model[grid_coord.x-1][grid_coord.y] > grid_coord.z
        self.left_blocked = (grid_coord.x<4 and self.curr_model[grid_coord.x+1][grid_coord.y] > grid_coord.z)

        if (self.back_blocked or self.front_blocked):
            if (self.left_blocked or self.right_blocked):
                if not self.left_blocked:
                    msg = "pg_hover_alternate"
                    self.push_flag = 3
                elif not self.right_blocked:
                    msg = "pg_hover_alternate"
                    self.push_flag = 4
                elif not self.back_blocked:
                    msg = "pg_hover"
                    self.push_flag = 1
                else:
                    msg = "pg_hover"
                    self.push_flag = 2
            else:
                msg = "pg_hover_alternate"
                self.push_flag = 0
        else:
            msg = "pg_hover"
            self.push_flag = 0
        print("Sending: ", msg)
        self.joints_pub.publish(msg)
        time.sleep(2)

        print('Push Flag:' + str(self.push_flag))

        # make query to ur5_arm_node and wait for callback
        self.query = "coordinates"
        self.query_pub.publish(self.query)
        time.sleep(2)

        # cmd_location = self.coord_trans(grid_coord)
        # add extra space for pushing
        real_coord.x += self.push_instruction[self.push_flag][0] * self.unit_length
        real_coord.y += self.push_instruction[self.push_flag][1] * self.unit_length

        # TODO pick up the block

        # go to x,y coordinates
        msg = str(real_coord.x) + ' ' + str(real_coord.y) + ' ' + str(self.curr_location[2])
        print('Sending:' + msg)
        self.coordinates_pub.publish(msg)
        time.sleep(2)

        # go to z coordinate and place the block
        msg = str(real_coord.x) + ' ' + str(real_coord.y) + ' ' + str(real_coord.z)
        print('Sending:' + msg)
        self.coordinates_pub.publish(msg)
        time.sleep(2)

        # TODO Publish to gripper to release

        # push the block into place
        if self.push_flag != 0:
            # TODO grip firmly
            self.push_block();

        # update the current model
        self.curr_model[grid_coord.x][grid_coord.y] += 1
        self.model_pub.publish(str(self.curr_model))

    def run(self):
        print("Path planner running")
        while not rospy.is_shutdown():
            try:
                # wait for user input
                if self.is_building:
                    # self.pickup()
                    block_index = 0
                    while block_index < len(self.grid_building):
                        # continue building until the build sequence is empty
                        self.place_block(self.grid_building[block_index], self.real_building[block_index])
                        block_index += 1
                    self.is_building = False
                    time.sleep(1)
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    pp = PathPlanner()
    pp.run()
