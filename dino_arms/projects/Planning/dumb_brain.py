#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import pandas as pd
import random
import time
from std_msgs.msg import String, Int16
from irl.msg import minimap, blocks

class Dumb_Brain(object):


    def __init__(self):
        rospy.init_node("dumb_brain")
        rospy.Subscriber("minimap", minimap, queue_size=10, callback=self.get_cubes)

        self.cubes = []
        self.cube_numbers = []

    def get_cubes(self, data):
        self.cubes = data.structure

        print "Here are the cubes I have, in no particular order:"
        for cube in self.cubes:
            print "Cube %s - x: %d, y: %d" %(cube.name, cube.x, cube.y)
            self.cube_numbers.append(int(cube.name))

        print self.cube_numbers



    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()




if __name__=="__main__":
    db = Dumb_Brain()
    db.run()
