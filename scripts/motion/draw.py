import rospy
import math
import st
import numpy as np
from std_msgs.msg import String
import Tkinter as tk
import time


class Drawer:
	def __init__(self):
		rospy.init.node('drawing stuff', anonymous = True)
		self.behaviorpub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
		self.armpub = rospy.Publisher('arm_cmd', String, queue_size=10)

	def DrawSquare(self, x, y, z):
		#getting into position
        center = [x, y, z]

		msg = "data: move_to:: " + x + "," + y + "," + (z+25)
        print "sending: ", msg
        self.armpub.publish(msg)

        #making the square
        msg = "data: move_to:: " + (x+50) + "," + (y+50) + "," + z
        print "sending: ", msg
        self.armpub.publish(msg)

        msg = "data: move_to:: " + (x-50) + "," + (y+50) + "," + z
        print "sending: ", msg
        self.armpub.publish(msg)

        msg = "data: move_to:: " + (x-50) + "," + (y-50) + "," + z
        print "sending: ", msg
        self.armpub.publish(msg)

        msg = "data: move_to:: " + (x+50) + "," + (y-50) + "," + z
        print "sending: ", msg
        self.armpub.publish(msg)

        msg = "data: move_to:: " + (x+50) + "," + (y+50) + "," + z
        print "sending: ", msg
        self.armpub.publish(msg)


        #picking up off the paper and finishing
        msg = "data: move_to:: " + x + "," + y + "," + (z+25)
        print "sending: ", msg
        self.armpub.publish(msg)


	def DrawCircle(self, x, y, z, r):
		#getting into position

		#drawing the circle

		#picking off the paper and finishing


	def Color(self, x, y, z, r):
		#getting into position

		#running algorithm that does a toddler coloring imitation

		#picking off the paper finishing


    def run(self):
         r = rospy.Rate(10)
         while not rospy.is_shutdown():
             r.sleep()

if __name__ == "__main__":
	draw = Drawer()
	draw.run()
	draw.DrawSquare()
	#draw.DrawCircle()
	#draw.Color()
	rospy.spin()
