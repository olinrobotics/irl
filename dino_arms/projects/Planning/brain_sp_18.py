#!/usr/bin/env python

"""
The Brain for Spring 2018

by Kevin Zhang

Currently a work in progress, MVP undergoing various testings

"""

import rospy
import rospkg
import numpy as np
import pandas as pd
import random
import time
from std_msgs.msg import String, Int16
from irl.msg import Cube, Structure
from Assembler.cube import Digital_Cube

from Assembler.assembly_instructor import Assembler

class Brain_Spring_2018(object):
    """
    the brain class, which holds almost all of the various components and from which
    all actions after perception will be executed
    """

    def __init__(self):
        self.asm = Assembler()

        rospy.init_node("brain_sp_18")
        rospy.Subscriber("test_run", String, queue_size=10, callback=self.test_run)
        rospy.Subscriber("/digital_env", Structure, self.asm.set_cube_list)

        self.digital_env_pub = rospy.Publisher("/digital_sig", String, queue_size=10)
        self.instructions_pub = rospy.Publisher("/assembly_instructions", Structure, queue_size=10)


    def test_run(self, data):
        """
        testing a digital environment with the assembler sequencer, might expand to include
        other modules as well
        """

        if data.data == "1st_stage":
            print "--------------STARTING A ROUND OF TRIAL------------------\n"
            self.digital_env_pub.publish("build")
            print "ENV BUILDING NEW STRUCTURE\n"
        elif data.data == "2nd_stage":
            print "ENV DONE, ASSEMBLER STARTING\n"
            msg = self.asm.sequence()
            print "SENDING INSTRUCTIONS"
            self.instructions_pub.publish(msg)
            print "DONE"

            print "-----------------------TRIAL FINISHED-------------------------\n"



    def run(self):
        """
        main run loop
        """

        print "Brain Spring 2018 is running"
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                r.sleep()
            except KeyboardInterrupt:
                print "\n Brain module turned off\n"
                break


if __name__=="__main__":
    b = Brain_Spring_2018()
    b.run()
