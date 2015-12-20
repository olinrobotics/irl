#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String
from edwin.msg import *
import time


class Drawer:
    def __init__(self):
        rospy.init_node('edwin_drawer', anonymous = True)
        rospy.Subscriber('/draw_cmd', Edwin_Shape, self.draw_callback, queue_size=10)

        self.behavior_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
        self.arm_pub.publish("data: set_speed:: 3000")

    def draw_callback(self, data):
        #getting into position
        motions = ["data: move_to:: " + str(data.x) + ", " + str(data.y) + ", " + str(data.z+250)+ ", " + str(0),
                    "data: rotate_hand:: " + str(200),
                    "data: rotate_wrist:: " + str(1000)]

        for motion in motions:
            print "sending: ", motion
            self.arm_pub.publish(motion)
            time.sleep(1)

        if data.shape == "square":
            self.draw_square(data.x, data.y, data.z)
        elif data.shape == "circle":
            self.draw_circle(data.x, data.y, data.z)
        else:
            print "ERROR: I can't draw that."

        #pick marker off paper
        msg = "data: move_to:: " + str(data.x) + ", " + str(data.y) + ", " + str(data.z+250) + ", " + str(0)
        print "sending: ", msg
        self.arm_pub.publish(msg)
        time.sleep(1)

    def draw_square(self, x, y, z):
        i = 0
        width = 200

        for i in range(5):
            if i == 0:
                msg = "data: move_to:: " + str(x+width) + ", " + str(y+width) + ", " + str(z) + ", " +str(0)
            elif i == 1:
                msg = "data: move_to:: " + str(x-width) + ", " + str(y+width) + ", " + str(z) + ", " +str(0)
            elif i == 2:
                msg = "data: move_to:: " + str(x-width) + ", " + str(y-width) + ", " + str(z) + ", " +str(0)
            elif i == 3:
                msg = "data: move_to:: " + str(x+width) + ", " + str(y-width) + ", " + str(z) + ", " +str(0)
            elif i == 4:
                msg = "data: move_to:: " + str(x+width) + ", " + str(y+width) + ", " + str(z) + ", " +str(0)

            print "sending: ", msg
            self.arm_pub.publish(msg)
            time.sleep(1)

    def draw_circle(self, x, y, z):
        #where x, y is the center of the circle, and r is predefined
        r = 200
        h = x
        k = y

        for xi in range(x-r, x+r, 50):
            yi = int(math.sqrt(r**2 - (xi-h)**2) + k)
            msg = "data: move_to:: " + str(xi) + ", " + str(yi) + ", " + str(z)+ ", " + str(0)
            print "publishing: ", msg
            self.arm_pub.publish(msg)
            time.sleep(.5)


        for xi in range(x+r, x-r-50, -50):
            yi = int(-1*(math.sqrt(r**2 - (xi-h)**2)) + k)
            msg = "data: move_to:: " + str(xi) + ", " + str(yi) + ", " + str(z)+ ", " + str(0)
            print "publishing: ", msg
            self.arm_pub.publish(msg)
            time.sleep(.5)

    #         #coloring the circle
    #         msg = "data: move_to:: " + str(x-200) + ", " + str(y) + ", " + str(z+250)+ ", " + str(0)
    #         print "sending: ", msg
    #         self.armpub.publish(msg)
    #         time.sleep(1)

    #         for i in range(-200, 200, 10):
    #             msg = "data: move_to:: " + str(x+i) + ", " + str(int(math.sqrt(40000-((x+i)**2)))+3500) + ", " + str(z)+ ", " + str(0)
    #             print "sending: ", msg
    #             self.armpub.publish(msg)
    #             time.sleep(.5)
    #             msg = "data: move_to:: " + str(x+i) + ", " + str(int(-1*math.sqrt(40000-((x+i)**2)))+3500) + ", " + str(z)+ ", " + str(0)
    #             print "sending: ", msg
    #             self.armpub.publish(msg)
    #             time.sleep(.5)


    def run(self):
        print "running"
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    draw = Drawer()
    draw.run()