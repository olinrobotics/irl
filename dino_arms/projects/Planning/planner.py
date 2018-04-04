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
from irl.msg import Cube, Structure, Cube_Structures
from std_msgs.msg import String, Int16
from Assembler.cube import Digital_Cube

from Assembler.assembly_instructor import Assembler

#TODO: import Ben's module

class Planner(object):
    """
    the planner class, which holds all of the components that comprise of the "brain" functionality in
    figuring out what to do with the data that perception provides, and then sends its instructions
    to the controller for physical execution
    """

    def __init__(self):
        self.asm = Assembler()

        # TODO: initialize Ben's class(es)

        self.cube_list = Structure() # the premlinary cube list input used to model the env
        self.cubes = Structure() # the cube list output used to sort the cubes
        self.two_structs = Cube_Structures()
        self.env_size = 5 # dimension of env
        self.current_env = np.empty((self.env_size,self.env_size,self.env_size), dtype=object) # the digital environment

        self.sorted_grid_cubes = None
        self.sorted_real_cubes = None

        rospy.init_node("planner")
        # rospy.Subscriber("test_run", String, queue_size=10, callback=self.test_run)
        # rospy.Subscriber("/digital_env", Structure, self.asm.set_cube_list)
        rospy.Subscriber("/perception", Structure, self.plan)

        # self.digital_env_pub = rospy.Publisher("/digital_sig", String, queue_size=10)
        self.instructions_pub = rospy.Publisher("/build_cmd", Cube_Structures, queue_size=10)


    def plan(self, cube_list):

        self.cube_list.building = cube_list.building

        # self.current_env = # TODO: call ben's method that will take self.cube_list and return a 3D array of the gridworld cubes
        #
        # self.add_descriptors()
        self.sorted_grid_cubes = self.sequence()

        # self.sorted_real_cubes = # TODO: call ben's method that will take sorted_grid_cubes and return a list of sorted cubes in the real world

        # self.two_structs.real_building = self.sorted_real_cubes
        self.two_structs.grid_building = self.sorted_grid_cubes

        self.instructions_pub.publish(self.two_structs)


    def add_descriptors(self):

        # make actual usable cubes from the environment and filling out all the information
        for x, y, z in itertools.product(*map(xrange,(self.env_size, self.env_size, self.env_size))):
            if self.current_env[x,y,z]:
                connections = 0
                for c in [[x+1, y],[x-1, y], [x,y+1], [x,y-1]]:
                    if all(n >= 0 and n < self.env_size for n in c) and self.current_env[c[0],c[1],z]:
                        connections += 1
                self.current_env[x,y,z].connections = connections
                self.current_env[x,y,z].height = self.current_env[x,y,z].z + 1
                self.cubes.building.append(self.current_env[x,y,z])


    def sequence(self):

        self.asm.set_cube_list(self.cube_list)
        return self.asm.sequence()

    # def test_run(self, data):
    #     """
    #     testing a digital environment with the assembler sequencer, might expand to include
    #     other modules as well
    #     """
    #
    #     if data.data == "1st_stage":
    #         print "--------------STARTING A ROUND OF TRIAL------------------\n"
    #         self.digital_env_pub.publish("build")
    #         print "ENV BUILDING NEW STRUCTURE\n"
    #     elif data.data == "2nd_stage":
    #         print "ENV DONE, ASSEMBLER STARTING\n"
    #         msg = self.asm.sequence()
    #         print "SENDING INSTRUCTIONS"
    #         self.instructions_pub.publish(msg)
    #         print "DONE"
    #
    #         print "-----------------------TRIAL FINISHED-------------------------\n"



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
