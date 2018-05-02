#!/usr/bin/env python

"""
Project Gemini Planner

by Kevin Zhang

Currently a work in progress, MVP undergoing various testings

"""

import rospy
import rospkg
import numpy as np
import pandas as pd
import random
import time
import itertools
from irl.msg import Real_Cube, Grid_Cube, Real_Structure, Grid_Structure, Cube_Structures
from std_msgs.msg import String, Int16
from Assembler.cube import Digital_Cube

from Assembler.assembly_instructor import Assembler
from Transformation.coordFrames import CoordFrames

class Planner(object):
    """
    the planner class, which holds all of the components that comprise of the "brain" functionality in
    figuring out what to do with the data that perception provides, and then sends its instructions
    to the controller for physical execution
    """

    def __init__(self):
        self.asm = Assembler()

        self.coord_trans = CoordFrames()
        # self.change_origin(-.065, .600, .0185)

        self.cube_list = Real_Structure() # the premlinary cube list input used to model the env
        self.cubes = Grid_Structure() # the cube list output used to sort the cubes
        self.two_structs = Cube_Structures()
        self.env_size = 5 # dimension of env
        self.current_env = np.empty((self.env_size,self.env_size,self.env_size), dtype=object) # the digital environment

        self.sorted_grid_cubes = None
        self.sorted_real_cubes = None

        rospy.init_node("instruction_planner")

        rospy.Subscriber("/perception", Real_Structure, self.plan)

        self.instructions_pub = rospy.Publisher("/build_cmd", Cube_Structures, queue_size=10)


    def change_origin(self, px, py, pz):

        origin_cube = Real_Cube()
        origin_cube.x = px
        origin_cube.y = py
        origin_cube.z = pz

        self.coord_trans.updateOrigin(origin_cube)


    def plan(self, cube_list):

        self.cube_list.building = cube_list.building

        self.current_env = self.coord_trans.convertBoard(self.cube_list)
        #
        self.add_descriptors()
        self.sorted_grid_cubes = self.sequence()
        #
        self.sorted_real_cubes = self.coord_trans.convertReal(self.sorted_grid_cubes)

        self.two_structs.real_building = self.sorted_real_cubes
        self.two_structs.grid_building = self.sorted_grid_cubes

        self.instructions_pub.publish(self.two_structs)


    def add_descriptors(self):

        self.cubes = Grid_Structure()
        current_env_center = np.empty((self.env_size,self.env_size,self.env_size), dtype=object)
        current_env_ring = np.empty((self.env_size,self.env_size,self.env_size), dtype=object)

        for x, y, z in itertools.product(*map(xrange,(self.env_size, self.env_size, self.env_size))):
            if x < 4 and x > 0 and y < 4 and y > 0:
                if self.current_env[x][y][z]:
                    current_env_center[x][y][z] = self.make_grid_cube(self.current_env[x][y][z])
            else:
                if self.current_env[x][y][z]:
                    current_env_ring[x][y][z] = self.make_grid_cube(self.current_env[x][y][z])

        # make actual usable cubes from the environment and filling out all the information
        for x, y, z in itertools.product(*map(xrange,(self.env_size, self.env_size, self.env_size))):
            if current_env_center[x][y][z]:
                connections = 0
                for c in [[x+1, y],[x-1, y], [x,y+1], [x,y-1]]:
                    if all(n >= 0 and n < self.env_size for n in c) and current_env_center[c[0]][c[1]][z]:
                        connections += 1
                current_env_center[x][y][z].connections = connections
                current_env_center[x][y][z].height = current_env_center[x][y][z].z + 1

            elif current_env_ring[x][y][z]:
                connections = 0
                for c in [[x+1, y],[x-1, y], [x,y+1], [x,y-1]]:
                    if all(n >= 0 and n < self.env_size for n in c) and current_env_ring[c[0]][c[1]][z]:
                        connections += 1
                current_env_ring[x][y][z].connections = connections
                current_env_ring[x][y][z].height = current_env_ring[x][y][z].z + 1

        for x, y, z in itertools.product(*map(xrange,(self.env_size, self.env_size, self.env_size))):
            if x < 4 and x > 0 and y < 4 and y > 0 and current_env_center[x][y][z]:
                self.cubes.building.append(current_env_center[x][y][z])
            elif current_env_ring[x][y][z]:
                self.cubes.building.append(current_env_ring[x][y][z])


    def make_grid_cube(self, cube):
        """
        converting between data types, this one converts from python cube to ros cube
        """
        grid_cube = Grid_Cube()
        grid_cube.x = cube.x
        grid_cube.y = cube.y
        grid_cube.z = cube.z
        return grid_cube


    def sequence(self):

        self.asm.set_cube_list(self.cubes)
        return self.asm.sequence()


    def run(self):
        """
        main run loop
        """

        print "Planner is running"
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                r.sleep()
            except KeyboardInterrupt:
                print "\n Planner module turned off\n"
                break


if __name__=="__main__":
    p = Planner()
    p.run()
