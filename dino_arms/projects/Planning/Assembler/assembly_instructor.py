#!/usr/bin/env python

"""
MVP assembler sequencer

by Kevin Zhang

takes in a list of building blocks that comprise of the structure being built, and
then returns a "sorted" list of building block instructions for how to go about
building that structure. uses formal systematic rule-based methods to determine
the sequence of blocks

this script is imported by Brain_Spring_2018 as package

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
from sequencer_RL import Smart_Sequencer


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
        self.RL = Smart_Sequencer()
        self.plane_mapping = {"[0, 0]":20,  "[1, 0]":13,  "[2, 0]":9, "[3, 0]":11,  "[4, 0]":15, \
                              "[0, 1]":21,  "[1, 1]":6,  "[2, 1]":1, "[3, 1]":5,  "[4, 1]":16, \
                              "[0, 2]":22,  "[1, 2]":2,  "[2, 2]":0, "[3, 2]":4, "[4, 2]":17, \
                              "[0, 3]":23,  "[1, 3]":7,  "[2, 3]":3, "[3, 3]":8,  "[4, 3]":18, \
                              "[0, 4]":24,  "[1, 4]":14,  "[2, 4]":10, "[3, 4]":12,  "[4, 4]":19 }


    def set_cube_list(self, cubes):
        """
        this is linked to Brain_Spring_2018, which when receiving
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


    def separate_cubes(self, cube_list):
        ring = []
        center = []
        for cube in cube_list:
            if cube.x < 4 and cube.x > 0 and cube.y < 4 and cube.y > 0:
                center.append(cube)
            else:
                ring.append(cube)

        return center, ring


    def sequence(self):
        """
        systematic sorting method
        the main sequence method, first sorts by height, then within each bin
        then sorts by connections and bins those into separate arrays,
        and at last sorts each binned connection type according to a known pattern above
        """

        # sort by height
        self.layers = self.sort_by_height(self.raw_sequence)
        layer_bin = []

        #TODO: figure out this flow of execution

        # segregate the heights into binned connections
        for layer in self.layers:
            # center, ring = self.separate_cubes(layer)
            # RL_bin = self.RL.smart_sequence(center)
            layer_bin.extend(self.sort_by_connections(layer))
        for connection in layer_bin:
            self.instructions.extend(self.sort_by_plane(connection))

        # print out instructions, and then return finished sequence to the brain
        self.print_sequence(self.instructions)
        return self.package_sequence(self.instructions)


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

    def sort_by_plane(self, connection):

        connection.sort(key=lambda x: self.plane_mapping[str([x.x, x.y])])
        return connection


    def package_sequence(self, instructions):
        """
        create a ros data structure to hold the information and return it back to
        the brain
        """

        msg = Grid_Structure()
        for cube in instructions:
            msg.building.append(self.make_real_cube(cube))

        return msg


    def print_sequence(self, instructions):
        """
        for debugging and convenience, prints the finished sequence
        """

        print "FINISHED INSTRUCTIONS ARE AS FOLLOWS"
        i = 1
        for instruction in instructions:
            print "Step", i, ": cube at", "x:", instruction.x, \
                                          "y:", instruction.y, \
                                          "z:", instruction.z
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
