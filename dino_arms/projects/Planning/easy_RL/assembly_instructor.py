#!/usr/bin/env python

"""
MVP assembler sequencer FOR USE IN RL

by Kevin Zhang

takes in a list of building blocks that comprise of the structure being built, and
then returns a "sorted" list of building block instructions for how to go about
building that structure. uses formal systematic rule-based methods to determine
the sequence of blocks

this script is imported by digital_env.py as package

how it works:
1. it merely sits and waits for a prompt from the brain executing its set_cube_list()
and giving it a list of cubes
2. brain will then call its sequence() method, by which it then correctly sequences the blocks
"""

import rospy
import numpy as np
import time
from std_msgs.msg import String, Int16
from irl.msg import Grid_Cube, Grid_Structure
from cube import Digital_Cube


class Assembler(object):
    """
    the assembler class, the main class of this script, holds the instructions and lists
    and also the sequencing methods
    """

    def __init__(self):
        self.vCube = np.vectorize(Digital_Cube) # faster struct using numpy
        self.cube_list = Grid_Structure() # custom ros data structure
        self.instructions = []  # the finished set of instructions
        self.raw_sequence = [] # the starting set of shuffled instructions
        self.layers = []    # a middle man for sequencing
        self.plane_mapping = {"[0, 0, 0]":0,  "[0, 1, 0]":1,  "[0, 2, 0]":2, \
                                "[1, 0, 0]":3,  "[1, 1, 0]":4,  "[1, 2, 0]":5, \
                                "[2, 0, 0]":6,  "[2, 1, 0]":7,  "[2, 2, 0]":8, \
                                "[0, 0, 1]":9,  "[0, 1, 1]":10,  "[0, 2, 1]":11, \
                                "[1, 0, 1]":12,  "[1, 1, 1]":13,  "[1, 2, 1]":14, \
                                "[2, 0, 1]":15,  "[2, 1, 1]":16,  "[2, 2, 1]":17, \
                                "[0, 0, 2]":18,  "[0, 1, 2]":19,  "[0, 2, 2]":20, \
                                "[1, 0, 2]":21,  "[1, 1, 2]":22,  "[1, 2, 2]":23, \
                                "[2, 0, 2]":24,  "[2, 1, 2]":25,  "[2, 2, 2]":26}


    def set_cube_list(self, cubes):
        """
        this is linked to a subscriber in Brain_Spring_2018, which when receiving
        a message will execute this method and populate the class with a list of cubes
        to be sorted
        """

        self.instructions = []
        self.raw_sequence = []
        self.cube_list.building = cubes.building
        for item in self.cube_list.building:
            self.raw_sequence.append(self.make_digital_cube(item))


    def make_digital_cube(self, cube):
        """
        converting between data types, this one converts from ros cube to python cube
        """

        digital_cube = self.vCube()
        digital_cube.height = cube.height
        digital_cube.connections = cube.connections
        digital_cube.x = cube.x
        digital_cube.y = cube.y
        digital_cube.z = cube.z
        return digital_cube


    def make_real_cube(self, cube):
        """
        converting between data types, this one converts from python cube to ros cube
        """

        real_cube = Grid_Cube()
        real_cube.height = cube.height
        real_cube.connections = cube.connections
        real_cube.x = cube.x
        real_cube.y = cube.y
        real_cube.z = cube.z
        return real_cube


    def sequence(self):
        """
        the main sequence method, first sorts by height, then within each bin
        sorts by connections,

        could potentially sort further in each connection bin in each height bin,
        but it was deemed not necessary
        """

        # sort by height
        self.layers = self.sort_by_height(self.raw_sequence)
        # within each height bin, sort by connections
        per_layer = []
        for layer in self.layers:
            per_layer.extend(self.sort_by_connections(layer))
        for layer in per_layer:
            self.instructions.extend(self.sort_by_plane(layer))

        # print out instructions, and then return finished sequence to the brain
        # self.print_sequence(self.instructions)
        return self.instructions


    def sort_by_height(self, cube_sequence):
        """
        sorts the whole cube sequence by height using sort with lambda
        then bins each height into separate arrays
        """

        # lambda sort by height
        cube_sequence.sort(key=lambda x: x.height)

        # binning
        binned_by_height = []
        layer = 1
        layer_bin = []
        for cube in cube_sequence:
            if cube.height == layer:
                layer_bin.append(cube)
            else:
                binned_by_height.append(layer_bin)
                layer_bin = []
                layer_bin.append(cube)
                layer += 1
        binned_by_height.append(layer_bin)
        return binned_by_height


    def sort_by_connections(self, layer):
        """
        for each binned height array, sort by number of connections of the cube
        with other cubes (how many faces are touching other cubes)

        for the purposes of RL, also bins the sorted cubes into different connections
        for easier targeting
        """

        layer.sort(key=lambda x: x.connections, reverse=True)

        binned_by_connections = []
        connections = 4
        connection_bin = []
        index = 0
        while index < len(layer):
            cube = layer[index]
            if cube.connections == connections:
                connection_bin.append(cube)
                index += 1
            else:
                if connection_bin != []:
                    binned_by_connections.append(connection_bin)
                connection_bin = []
                connections -= 1
        binned_by_connections.append(connection_bin)
        return binned_by_connections


    def sort_by_plane(self, layer):

        layer.sort(key=lambda x: self.plane_mapping[str([x.x, x.y, x.z])])
        return layer

    def print_sequence(self, instructions):
        """
        for debugging and convenience, prints the finished sequence
        """

        print "FINISHED INSTRUCTIONS ARE AS FOLLOWS"
        i = 1
        for instruction in instructions:
            print "Connections:", instruction[0].connections, "Height:", instruction[0].height
            for item in instruction:
                print "Step", i, ": cube at", "x:", item.x, \
                                              "y:", item.y, \
                                              "z:", item.z
                i += 1

        print "THOSE ARE ALL THE INSTRUCTIONS"


    def run(self):
        """
        main run loop
        """

        print "Assembly Sequencer is running"
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                r.sleep()
            except KeyboardInterrupt:
                print "\n Assembler module turned off"
                break


if __name__=="__main__":
    asm = Assembler()
    asm.run()
