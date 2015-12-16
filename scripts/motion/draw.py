#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String
import time


class Drawer:
    def __init__(self):
        rospy.init_node('drawing_stuff', anonymous = True)
        self.behaviorpub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.armpub = rospy.Publisher('arm_cmd', String, queue_size=10)
        self.armpub.publish("data: set_speed:: 3000")



    def DrawSquare(self, x, y, z):
        time.sleep(3)
        #getting into position

        msg = "data: move_to:: " + str(x) + "," + str(y) + "," + str(z+250) + ", " + str(0)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)

        msg = "data: rotate_hand:: " + str(200)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)

        msg = "data: rotate_wrist:: " + str(1000)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)

        #making the square
        msg = "data: move_to:: " + str(x+250) + ", " + str(y+250) + ", " + str(z)+ ", " + str(0)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)

        msg = "data: move_to:: " + str(x-250) + ", " + str(y+250) + ", " + str(z)+ ", " + str(0)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)

        msg = "data: move_to:: " + str(x-250) + ", " + str(y-250) + ", " + str(z)+ ", " + str(0)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)

        msg = "data: move_to:: " + str(x+250) + ", " + str(y-250) + ", " + str(z)+ ", " + str(0)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)

        msg = "data: move_to:: " + str(x+250) + ", " + str(y+250) + ", " + str(z)+ ", " + str(0)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)


        #picking up off the paper and finishing
        msg = "data: move_to:: " + str(x) + ", " + str(y) + ", " + str(z+250)+ ", " + str(0)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)




    def DrawCircle(self, x, y, z):
        time.sleep(3)
        #getting into position
        msg = "data: move_to:: " + str(x) + ", " + str(y) + ", " + str(z+250)+ ", " + str(0)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)

        msg = "data: rotate_hand:: " + str(200)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)

        msg = "data: rotate_wrist:: " + str(1000)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)

        #drawing the circle
        msg = "data: move_to:: " + str(x) + ", " + str(y) + ", " + str(z)+ ", " + str(0)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)

        for i in range(-200, 200, 10):
            msg = "data: move_to:: " + str(x+i) + ", " + str(int(math.sqrt(40000-((x+i)**2)))) + ", " + str(z)+ ", " + str(0)
            print "sending: ", msg
            self.armpub.publish(msg)
            time.sleep(1)

        for j in range(-200, 200, 10):
            msg = "data: move_to:: " + str(x-i) + ", " + str(int(-1*math.sqrt(40000-((x-i)**2)))) + ", " + str(z)+ ", " + str(0)
            print "sending: ", msg
            self.armpub.publish(msg)
            time.sleep(1)

        #picking off the paper and finishing
        msg = "data: move_to:: " + str(x) + ", " + str(y) + ", " + str(z+250)+ ", " + str(0)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)




    def Color(self, x, y, z):
        time.sleep(3)	
        #getting into position
        msg = "data: move_to:: " + str(x) + ", " + str(y) + ", " + str(z+250)+ ", " + str(0)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)

        msg = "data: rotate_hand:: " + str(2000)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)

        msg = "data: rotate_wrist:: " + str(1000)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)

        #coloring the circle
        msg = "data: move_to:: " + str(x) + ", " + str(y) + ", " + str(z)+ ", " + str(0)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)

        for i in range(-1000, 1000, 10):
            msg = "data: move_to:: " + str(x+i) + ", " + str(int(math.sqrt(1000000-((x+i)**2)))) + ", " + str(z)+ ", " + str(0)
            print "sending: ", msg
            self.armpub.publish(msg)
            time.sleep(1)
            msg = "data: move_to:: " + str(x+i) + ", " + str(int(-1*math.sqrt(1000000-((x+i)**2)))) + ", " + str(z)+ ", " + str(0)
            print "sending: ", msg
            self.armpub.publish(msg)
            time.sleep(1)

        #picking off the paper and finishing
        msg = "data: move_to:: " + str(x) + ", " + str(y) + ", " + str(z+250)+ ", " + str(0)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)



    def run(self):
         r = rospy.Rate(10)
         while not rospy.is_shutdown():
             r.sleep()



if __name__ == "__main__":
    draw = Drawer()
    #draw.run()
    #draw.DrawSquare(0, 3500, -670)
    draw.DrawCircle(0, 3500, -670)
    #draw.Color(0, 3500, -670)
    rospy.spin()
