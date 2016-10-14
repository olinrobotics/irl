#!/usr/bin/env python
import rospy
import cv2
import cv2.cv as cv
import numpy as np
import random
import time
import math
import operator
import itertools

from std_msgs.msg import String
from sensor_msgs.msg import Image
from edwin.msg import Edwin_Shape
from cv_bridge import CvBridge, CvBridgeError

class Gamei:
	def __init__(self):
		self.draw_pub = rospy.Publisher('draw_cmd', Edwin_Shape, queue_size=10)
		self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
		self.behav_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)

	def img_callback(self, data):
		try:
			self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)


	def field_scan(self):
		time.sleep(5)

		cv2.imshow("img", self.frame)
		c = cv2.waitKey(1)

	def run(self):
	 	time.sleep(5)
	 	print "running"

		self.arm_pub.publish("data: set_speed:: 1000")
		print "set speed to 1000"
		time.sleep(1)

		self.arm_pub.publish("data: set_accel:: 100")
		print "set accel to 100"
		time.sleep(1)


		#Player = 0
        #Edwin  = 1

        running = True
		# turn = random.randint(0,1)
		turn = 0
		if turn == 0:
			print "YOUR TURN"
			self.behav_pub.publish("nudge")
			time.sleep(2)

		ai = False
		while running:
 	 		if turn == 0: #Player turn.
	 	         print "player's turn"

	 		elif turn == 1: #Edwin's turn
	 			print "edwin's turn"

		cv2.destroyAllWindows()
		print "Finished with Simon Says, hope you enjoyed :)"

if __name__ == '__main__':
	rospy.init_node('ss_gamemaster', anonymous = True)
	gm = Game()
	gm.run()
