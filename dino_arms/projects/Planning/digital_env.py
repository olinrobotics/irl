#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String, Int16
import time
from irl.msg import cubes, structure
from cube import Cube



class Environment(object):

    def __init__(self):
        self.vCube = np.vectorize(Cube)
        self.env = np.empty((5,5,5), dtype=object)
        for i in range(5):
            for j in range(5):
                for k in range(5):
                    self.env[i,j,k] = self.vCube()
        self.cubes = []
        self.build_prob = 0.7

        rospy.init_node("environment")
        self.pub = rospy.Publisher("digital_env", structure, queue_size=10)
        rospy.Subscriber("digital_sig", String, self.create_a_struct)

    def reset(self):
        for i in range(5):
            for j in range(5):
                for k in range(5):
                    self.env[i,j,k].turn_off()
        self.cubes = []
        self.build_prob = 0.7


    def build_struct(self):

        layer = 0
        while layer < 5:
            self.build_prob -= 0.1
            for i in range(5):
                for j in range(5):
                    if  layer == 0 or self.env[i,j,layer-1].activated:
                        prob = np.random.random_sample()
                        if prob < self.build_prob:
                            self.env[i,j,layer].turn_on(layer, i, j, layer)
            layer += 1

        for i in range(5):
            for j in range(5):
                for k in range(5):
                    if self.env[i,j,k].activated:
                        connections = 0
                        connections += 1 if i-1 >= 0 and self.env[i-1,j, k].activated else 0
                        connections += 1 if i+1 < 5 and self.env[i+1,j, k].activated else 0
                        connections += 1 if j-1 >= 0 and self.env[i,j-1, k].activated else 0
                        connections += 1 if j+1 < 5 and self.env[i,j+1, k].activated else 0
                        self.env[i,j,k].set_connectivity(connections)
                        self.cubes.append(self.make_real_cube(self.env[i,j,k]))

    def make_real_cube(self, cube):
        real_cube = cubes()
        real_cube.height = cube.height
        real_cube.connections = cube.connections
        real_cube.x = cube.x
        real_cube.y = cube.y
        real_cube.z = cube.z
        return real_cube


    def send_struct(self):
        np.random.shuffle(self.cubes)
        struct = structure()
        for block in self.cubes:
            struct.structure.append(block)

        print "SENDING BUILDING BLOCKS"
        self.pub.publish(struct)
        print "DONE"



    def print_struct(self):
        column = 0
        printed_map = np.zeros((5,5))
        for i in range(5):
            for j in range(5):
                k = 0
                while k < 5 and self.env[i,j,k].activated:
                    k += 1
                printed_map[i,j] = k

        print "STRUCT AS SEEN FROM BIRD EYE VIEW"
        for  i in range(5):
            print np.array2string(printed_map[i,:])[1:-1]


    def create_a_struct(self, data):
        if data.data == "build":
            self.reset()
            self.build_struct()
            self.print_struct()
            self.send_struct()


    def run(self):
        print "Digital Environment is running"
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                r.sleep()
            except KeyboardInterrupt:
                print "\n Environment module turned off"
                break



if __name__=="__main__":
    env = Environment()
    env.run()
