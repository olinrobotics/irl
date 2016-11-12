#!/usr/bin/env python
import rospy
import math
# import st
import numpy as np
from std_msgs.msg import String
from edwin.msg import *
import time

def z_calculation(input_y):
	scaler = -735 - int((input_y - 4000)/9.4)
	return scaler


def run():
	rospy.init_node('arm_tester', anonymous=True)
	pub = rospy.Publisher('write_cmd', Edwin_Shape, queue_size=10)
	arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
	time.sleep(1)
	print "starting"

	# while not rospy.is_shutdown():
	msg = Edwin_Shape()

	msg.shape = "defgh"
	msg.x = -300
	msg.y = 4200
	#-770 for writing
	msg.z = -795

	pub.publish(msg)
	time.sleep(5)
        # # #Board X and Y positions
        # b_x = 0
        # b_y = 4000
        # b_w = 250
		#
        # msg.shape = "board"
        # msg.x = b_x
        # msg.y = b_y
        # msg.z = -743
        # pub.publish(msg)
        # time.sleep(10)
		#
        # #Sector centroids
        # b_centers = {}
        # b_centers[0] = (b_x - 2.25*b_w, b_y + 1.5*b_w)
        # b_centers[1] = (b_x, b_y + 1.5*b_w)
        # b_centers[2] = (b_x + 2*b_w, b_y + 1.5*b_w)
		#
        # b_centers[3] = (b_x - 2*b_w, b_y)
        # b_centers[4] = (b_x, b_y)
        # b_centers[5] = (b_x + 2*b_w, b_y)
		#
        # b_centers[6] = (b_x - 2*b_w, b_y - 1.5*b_w)
        # b_centers[7] = (b_x, b_y - 1.5*b_w)
        # b_centers[8] = (b_x + 2*b_w, b_y - 1.5*b_w)
		#
        # for b in range(9):
        #     if b in [0, 2]:
        #         msg.z = -795
        #     elif b == 1:
        #         msg.z = -790
        #     elif b < 6:
        #         msg.z = -750
        #     else:
        #         msg.z = -720
        #     msg.shape = "square"
        #     msg.x = b_centers[b][0]
        #     msg.y = b_centers[b][1]
        #     pub.publish(msg)
        #     time.sleep(15)

if __name__ == '__main__':
	run()
