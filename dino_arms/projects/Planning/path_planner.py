#!/usr/bin/env python

"""
Project Gemini planner

By Yichen Jiang

Path Planner and Executer for block placement.
"""

import rospy
import rospkg
import numpy as np
import time
import sys
import math
from irl.msg import Cube_Structures, Grid_Cube, Real_Cube, Real_Structure, Grid_Structure
from std_msgs.msg import String, Bool
rospack = rospkg.RosPack()
PACKAGE_PATH = rospack.get_path("irl")
sys.path.append(PACKAGE_PATH + '/projects/Controls')
from ur5_arm_node import Arm

# python path_planner.py _robot_ip:=10.42.0.54
# pollux: 10.42.0.175, castor: 10.42.0.54
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
        # build_cmd is rececived from the brain as a cube structures with real and grid coordinates
        self.cmd_sub = rospy.Subscriber("/build_cmd", Cube_Structures, self.cmd_callback,queue_size=1)

        # setting up publisher for two arms
        # two ways of controlling the arm with coordinates / joint behaviors
        self.coordinates_pub_pollux = rospy.Publisher("/coordinates_cmd_pollux", String, queue_size=10)
        self.joints_pub_pollux = rospy.Publisher("/behaviors_cmd_pollux", String, queue_size=10)
        # Make query about the joints/coordinates information and wait for callback
        self.query_pub_pollux = rospy.Publisher("/query_cmd_pollux", String, queue_size=10)
        self.info_sub_pollux = rospy.Subscriber("/arm_info_pollux", String, self.info_callback, queue_size=10)
        self.arm_status_pollux = rospy.Subscriber("/arm_status_pollux", String, self.pollux_status_callback, queue_size=10)

        # two ways of controlling the arm with coordinates / joint behaviors
        self.coordinates_pub_castor = rospy.Publisher("/coordinates_cmd_castor", String, queue_size=10)
        self.joints_pub_castor = rospy.Publisher("/behaviors_cmd_castor", String, queue_size=10)
        # Make query about the joints/coordinates information and wait for callback
        self.query_pub_castor = rospy.Publisher("/query_cmd_castor", String, queue_size=10)
        self.info_sub_castor = rospy.Subscriber("/arm_info_castor", String, self.info_callback, queue_size=10)
        self.arm_status_castor = rospy.Subscriber("/arm_status_castor", String, self.castor_status_callback, queue_size=10)

        # controller_status determines when Perception start looking for a new goal
        self.status_pub = rospy.Publisher("/controller_status", Bool, queue_size=10)

        self.grip_pub = rospy.Publisher("/grip_cmd", String, queue_size=10)

        self.curr_model = [[0 for col in range(5)] for row in range(5)]
        self.is_building = False
        self.castor_busy = False
        self.pollux_busy = False

        # geometry of the cube
        self.unit_length = 0.1016

        self.grid_building = Grid_Structure()
        self.real_building = Real_Structure()
        self.query = ""
        self.curr_location = []
        self.curr_angle = []
        self.push_flag = 0 # 0:don't need pushing; 1:push from back; 2:push from front; 3:push from left; 4: push from right
        self.push_instruction = [(0,0), (0, -1.0/3), (0,1.0/3), (-1.0/3, 0), (1.0/3, 0)]

    def cmd_callback(self,data):
        '''
        Parse the build command from the brain
        '''
        self.grid_building = data.grid_building
        self.real_building = data.real_building
        self.is_building = True
        self.status_pub.publish(True)

    def info_callback(self,data):
        '''
        Parse realtime coordinates / joint angles of the arm
        '''
        if self.query == "coordinates":
            arm_info = data.data[1:len(data.data)-1]
            print("Arm coordinates are: " + arm_info)
            self.curr_location = [float(i) for i in arm_info.split(',')]
        elif self.query == "joints":
            arm_info = data.data[1:len(data.data)-1]
            # print("Joing angles are: " + arm_info)
            self.curr_angle = [float(i) for i in arm_info.split(',')]

    def castor_status_callback(self, data):
        '''
        check if castor is busy
        '''
        if data.data == "busy":
            self.castor_busy = True
        else:
            self.castor_busy = False

    def pollux_status_callback(self, data):
        '''
        check if pollux is busy
        '''
        if data.data == "busy":
            self.pollux_busy = True
        else:
            self.pollux_busy = False

    def check_castor(self):
        time.sleep(0.5)
        while self.castor_busy:
            pass
        time.sleep(1)

    def check_pollux(self):
        time.sleep(0.5)
        while self.pollux_busy:
            pass
        time.sleep(1)

    def pickup(self):
        '''
        Pick up the cube from a predefined location
        pick-up locations will be mirrored for two arms
        '''
        pass

    def coord_trans(self, base):
        '''
        Transform base coordinates to actual coordinate for the arm to place the block
        Each cube has the dimension of 101.6mm. Assume the default location is 110.29, -372.42, 289.06 for now
        zero z is -191.0
        Not used anymore
        '''
        default = [0.1103, -0.3718, 0.2890]
        default[1] = default[1] + 2 * self.unit_length
        # TODO redefine zero of z

        real_x = default[0] - (base[0]-2) * self.unit_length + self.push_instruction[self.push_flag][0] * self.unit_length
        real_y = default[1] - base[1] * self.unit_length + self.push_instruction[self.push_flag][1] * self.unit_length
        real_z = default[2] + base[2] * self.unit_length
        return [real_x, real_y, real_z]

    def push_block(self, grid_coord):
        '''
        Pushing building blocks into desired location when necessary
        '''
        # make query to ur5 arm for current coordinates
        self.query = "coordinates"
        if grid_coord.x<2:
            self.query_pub_pollux.publish(self.query)
        else:
            self.query_pub_castor.publish(self.query)
        time.sleep(2)

        # go up 2 units
        self.curr_location[2] = self.curr_location[2] + 2 * self.unit_length;
        msg = str(self.curr_location[0]) + ' ' + str(self.curr_location[1]) + ' ' + str(self.curr_location[2])
        if grid_coord.x<2:
            print('Sending Pollux:' + msg)
            self.coordinates_pub_pollux.publish(msg)
            self.check_pollux()
        else:
            print('Sending Castor:' + msg)
            self.coordinates_pub_castor.publish(msg)
            self.check_castor()

        # go outward
        self.curr_location[0] = self.curr_location[0] + self.push_instruction[self.push_flag][0] * 4 * self.unit_length;
        self.curr_location[1] = self.curr_location[1] + self.push_instruction[self.push_flag][1] * 4 * self.unit_length;
        msg = str(self.curr_location[0]) + ' ' + str(self.curr_location[1]) + ' ' + str(self.curr_location[2])
        if grid_coord.x<2:
            print('Sending Pollux:' + msg)
            self.coordinates_pub_pollux.publish(msg)
            self.check_pollux()
        else:
            print('Sending Castor:' + msg)
            self.coordinates_pub_castor.publish(msg)
            self.check_castor()

        # go down 2 units
        self.curr_location[2] = self.curr_location[2] - 2 * self.unit_length;
        msg = str(self.curr_location[0]) + ' ' + str(self.curr_location[1]) + ' ' + str(self.curr_location[2])
        if grid_coord.x<2:
            print('Sending Pollux:' + msg)
            self.coordinates_pub_pollux.publish(msg)
            self.check_pollux()
        else:
            print('Sending Castor:' + msg)
            self.coordinates_pub_castor.publish(msg)
            self.check_castor()

        # push inward
        self.curr_location[0] = self.curr_location[0] - self.push_instruction[self.push_flag][0] * 2 * self.unit_length;
        self.curr_location[1] = self.curr_location[1] - self.push_instruction[self.push_flag][1] * 2 * self.unit_length;
        msg = str(self.curr_location[0]) + ' ' + str(self.curr_location[1]) + ' ' + str(self.curr_location[2])
        if grid_coord.x<2:
            print('Sending Pollux:' + msg)
            self.coordinates_pub_pollux.publish(msg)
            self.check_pollux()
        else:
            print('Sending Castor:' + msg)
            self.coordinates_pub_castor.publish(msg)
            self.check_castor()

    def place_block(self, grid_coord, real_coord):
        '''
        Go to universal starting position and place blocks in provided order
        turn the wrist 90 degrees if other blocks are in the way and use pushing to place cube at tricky position
        pg_hover: 90, -90, 45, -45, -90, 0
        coor : 110.29 -372.42 289.06
        '''

        self.back_blocked = (grid_coord.y<4 and self.curr_model[grid_coord.x][grid_coord.y+1] > grid_coord.z)
        self.front_blocked = (grid_coord.y>0 and self.curr_model[grid_coord.x][grid_coord.y-1] > grid_coord.z)
        self.right_blocked = (grid_coord.x>0 and self.curr_model[grid_coord.x-1][grid_coord.y] > grid_coord.z)
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

            # TODO: new pg_hover joints: 90 -104.47 93.78 -78.7 -90 0
            self.push_flag = 0
        print("Sending: ", msg)
        print(grid_coord.y)
        if grid_coord.x<2:
            self.joints_pub_pollux.publish(msg)
            self.check_pollux()
        else:
            self.joints_pub_castor.publish(msg)
            self.check_castor()

        print('Push Flag:' + str(self.push_flag))

        # make query to ur5_arm_node and wait for callback
        self.query = "coordinates"
        if grid_coord.x<2:
            self.query_pub_pollux.publish(self.query)
        else:
            self.query_pub_castor.publish(self.query)
        time.sleep(2)

        # cmd_location = self.coord_trans(grid_coord)
        # add extra space for pushing
        real_coord.x += self.push_instruction[self.push_flag][0] * self.unit_length
        real_coord.y += self.push_instruction[self.push_flag][1] * self.unit_length

        # TODO pick up the block

        # max_z = 0.47
        print("The block to build is at: " + str(grid_coord.x) + ', ' + str(grid_coord.y) + ', ' + str(grid_coord.z))

        # go to x,y coordinates
        print("move xy")
        print(self.curr_location)
        msg = str(real_coord.x) + ' ' + str(real_coord.y) + ' ' + str(self.curr_location[2])
        if grid_coord.x<2:
            print('Sending Pollux:' + msg)
            self.coordinates_pub_pollux.publish(msg)
            self.check_pollux()
        else:
            print('Sending Castor:' + msg)
            self.coordinates_pub_castor.publish(msg)
            self.check_castor()

        # go to z coordinate and place the block
        print("move z")
        msg = str(real_coord.x) + ' ' + str(real_coord.y) + ' ' + str(real_coord.z)
        if grid_coord.x<2:
            print('Sending Pollux:' + msg)
            self.coordinates_pub_pollux.publish(msg)
            self.check_pollux()
        else:
            print('Sending Castor:' + msg)
            self.coordinates_pub_castor.publish(msg)
            self.check_castor()

        # TODO Publish to gripper to release

        # push the block into place
        if self.push_flag != 0:
            # TODO grip firmly
            self.push_block(grid_coord);

        # update the current model
        self.curr_model[grid_coord.x][grid_coord.y] += 1
        self.model_pub.publish(str(self.curr_model))

    def run(self):
        print("Path planner running")
        self.status_pub.publish(False)
        while not rospy.is_shutdown():
            try:
                # wait for user input
                if self.is_building:
                    # self.pickup()
                    block_index = 0
                    while block_index < len(self.grid_building.building):
                        # continue building until the build sequence is empty
                        self.place_block(self.grid_building.building[block_index], self.real_building.building[block_index])
                        block_index += 1
                    self.is_building = False
                    self.status_pub.publish(False)
                    time.sleep(1)
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    pp = PathPlanner()
    pp.run()
