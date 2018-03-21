#!/usr/bin/env python

import rospy
import numpy as np
import time
from std_msgs.msg import String, Int16
from irl.msg import Cube, Structure
from cube import Digital_Cube


class Assembler(object):

    def __init__(self):
        self.vCube = np.vectorize(Digital_Cube)
        self.cube_list = Structure()
        self.instructions = []
        self.raw_sequence = []
        self.layers = []

        rospy.init_node("assembler")
        rospy.Subscriber("/digital_env", Structure, self.set_cube_list)
        self.pub = rospy.Publisher("/assembly_instructions", Structure, queue_size=10)


    def set_cube_list(self, cubes):
        self.instructions = []
        self.raw_sequence = []
        self.cube_list.building = cubes.building
        for item in self.cube_list.building:
            self.raw_sequence.append(self.make_digital_cube(item))
        self.sequence()


    def make_digital_cube(self, cube):
        digital_cube = self.vCube()
        digital_cube.height = cube.height
        digital_cube.connections = cube.connections
        digital_cube.x = cube.x
        digital_cube.y = cube.y
        digital_cube.z = cube.z
        return digital_cube


    def make_real_cube(self, cube):
        real_cube = Cube()
        real_cube.height = cube.height
        real_cube.connections = cube.connections
        real_cube.x = cube.x
        real_cube.y = cube.y
        real_cube.z = cube.z
        return real_cube


    def sequence(self):
        self.layers = self.sort_by_height(self.raw_sequence)
        for layer in self.layers:
            self.instructions.extend(self.sort_by_connections(layer))

        self.print_sequence(self.instructions)
        self.export_sequence(self.instructions)


    def sort_by_height(self, cube_sequence):
        cube_sequence.sort(key=lambda x: x.height)

        binned_by_height = []
        layer = 0
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
        layer.sort(key=lambda x: x.connections, reverse=True)
        return layer


    def export_sequence(self, instructions):
        msg = Structure()
        for cube in instructions:
            msg.building.append(self.make_real_cube(cube))

        print "SENDING INSTRUCTIONS"
        self.pub.publish(msg)
        print "DONE"


    def print_sequence(self, instructions):
        print "FINISHED INSTRUCTIONS ARE AS FOLLOWS"
        i = 1
        for instruction in instructions:
            print "Step", i, ": cube at", instruction.x, instruction.y, instruction.z
            i += 1

        print "THOSE ARE ALL THE INSTRUCTIONS"


    def run(self):
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
