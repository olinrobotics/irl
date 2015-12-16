#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String
import time


class Drawer:
	def __init__(self):
		rospy.init.node('drawing stuff', anonymous = True)
        self.pub.publish("data: set_speed:: 3000")
		self.behaviorpub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
		self.armpub = rospy.Publisher('arm_cmd', String, queue_size=10)



	def DrawSquare(self, x, y, z):
		#getting into position


		msg = "data: move_to:: " + x + "," + y + "," + (z+250)
        print "sending: ", msg
        self.armpub.publish(msg)

        msg = "data: rotate_hand:: " + 200
        print "sending: ", msg
        self.armpub.publish(msg)

        msg = "data: rotate_wrist:: " + 1000
        print "sending: ", msg
        self.armpub.publish(msg)

        #making the square
        msg = "data: move_to:: " + (x+250) + ", " + (y+250) + ", " + z
        print "sending: ", msg
        self.armpub.publish(msg)

        msg = "data: move_to:: " + (x-250) + ", " + (y+250) + ", " + z
        print "sending: ", msg
        self.armpub.publish(msg)

        msg = "data: move_to:: " + (x-250) + ", " + (y-250) + ", " + z
        print "sending: ", msg
        self.armpub.publish(msg)

        msg = "data: move_to:: " + (x+250) + ", " + (y-250) + ", " + z
        print "sending: ", msg
        self.armpub.publish(msg)

        msg = "data: move_to:: " + (x+250) + ", " + (y+250) + ", " + z
        print "sending: ", msg
        self.armpub.publish(msg)


        #picking up off the paper and finishing
        msg = "data: move_to:: " + x + ", " + y + ", " + (z+250)
        print "sending: ", msg
        self.armpub.publish(msg)




	def DrawCircle(self, x, y, z):
		#getting into position
        msg = "data: move_to:: " + x + ", " + y + ", " + (z+250)
        print "sending: ", msg
        self.armpub.publish(msg)

        msg = "data: rotate_hand:: " + 200
        print "sending: ", msg
        self.armpub.publish(msg)

        msg = "data: rotate_wrist:: " + 1000
        print "sending: ", msg
        self.armpub.publish(msg)

		#drawing the circle
        msg = "data: move_to:: " + x + ", " + y + ", " + z
        print "sending: ", msg
        self.armpub.publish(msg)

        for i in range(-1000, 1000, 10):
            msg = "data: move_to:: " + (x+i) + ", " + (math.sqrt(1000000-(x^2))) + ", " + z
            print "sending: ", msg
            self.armpub.publish(msg)
        
        for j in range(-1000, 1000, 10):
            msg = "data: move_to:: " + (x-i) + ", " + (-1*math.sqrt(1000000-(x^2))) + ", " + z
            print "sending: ", msg
            self.armpub.publish(msg)

		#picking off the paper and finishing
        msg = "data: move_to:: " + x + ", " + y + ", " + (z+250)
        print "sending: ", msg
        self.armpub.publish(msg)




	def Color(self, x, y, z):	
        #getting into position
        msg = "data: move_to:: " + x + ", " + y + ", " + (z+250)
        print "sending: ", msg
        self.armpub.publish(msg)

        msg = "data: rotate_hand:: " + 200
        print "sending: ", msg
        self.armpub.publish(msg)

        msg = "data: rotate_wrist:: " + 1000
        print "sending: ", msg
        self.armpub.publish(msg)

        #coloring the circle
        msg = "data: move_to:: " + x + ", " + y + ", " + z
        print "sending: ", msg
        self.armpub.publish(msg)

        for i in range(-1000, 1000, 10):
            msg = "data: move_to:: " + (x+i) + ", " + (math.sqrt(1000000-(x^2))) + ", " + z
            print "sending: ", msg
            self.armpub.publish(msg)
            msg = "data: move_to:: " + (x+i) + ", " + (-1*math.sqrt(1000000-(x^2))) + ", " + z
            print "sending: ", msg
            self.armpub.publish(msg)

        #picking off the paper and finishing
        msg = "data: move_to:: " + x + ", " + y + ", " + (z+250)
        print "sending: ", msg
        self.armpub.publish(msg)



    def run(self):
         r = rospy.Rate(10)
         while not rospy.is_shutdown():
             r.sleep()



if __name__ == "__main__":
	draw = Drawer()
	draw.run()
	draw.DrawSquare(0, 3500, -670)
	#draw.DrawCircle(x, y, z)
	#draw.Color(x, y, z)
	rospy.spin()
