#!/usr/bin/env python
"""
This is the demo script for the SimonSays module. It requires theses modules to be running:

rosrun edwin arm_node.py
rosrun edwin arm_behaviors.py
rosrun edwin tts_engine.py
rosrun edwin stt_engine.py
"""

import rospy
import cv2
import cv2.cv as cv
import numpy as np
import random
import time
import math
from collections import namedtuple

from std_msgs.msg import String
from sensor_msgs.msg import Image
from edwin.msg import Edwin_Shape
from cv_bridge import CvBridge, CvBridgeError

EDWIN_NAME = "edwin"
USER_NAME = "user"

class Game:
	def __init__(self, max_turns = 5):
		self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
		self.behav_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)

		self.SimonCommand = namedtuple("SimonCommand", ["simon_ID", "simon_says", "command"])

		self.max_turns = max_turns

	def img_callback(self, data):
		try:
			self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

	def field_scan(self):
		time.sleep(5)

		cv2.imshow("img", self.frame)
		c = cv2.waitKey(1)

	def issue_simon_cmd(self):
		"""If Simon is Edwin:

		This function issues a Simon command.
		It is said outloud, so depends on the tts_engine to be running"""
		pass

	def check_simon_response(self):
		"""If Simon is Edwin:

		This function checks to see if the players have followed Edwin's cmd
		This relies on skeleton tracker to be functional
		"""
		pass


	def listen_for_simon(self):
		"""If Simon is the User:

		This function listens to a spoken Simon command from user
		This relies on stt_engine to be running"""
		pass

	def follow_simon_cmd(self):
		"""If Simon is the User:

		This function takes the command from listen_for_simon() and interprets
		The resulting command is sent to arm_node for physical motion"""
		pass



	def run(self):
		"""Game mainloop. Runs for as long as max_turns is defined"""

	 	time.sleep(5)
	 	print "running"

		self.simon_ID = EDWIN_NAME
		autonomous_play = False

		#two phases of simon says, issue command, check for follower response
		turn_count = 0
		while turn_count < self.max_turns:
			turn_count += 1

			#if playing autonomously, then Edwin issues commands and follows his own
			if autonomous_play:
				self.issue_simon_cmd()
				self.follow_simon_cmd()
				continue

			#issue command
			if self.simon_ID == EDWIN_NAME:
				self.issue_simon_cmd()
			else:
				self.listen_for_simon()

			#check for response
			if self.simon_ID == EDWIN_NAME:
				self.check_simon_response()
			else:
				self.follow_simon_cmd()

		cv2.destroyAllWindows()
		print "Finished with Simon Says, hope you enjoyed :)"

if __name__ == '__main__':
	rospy.init_node('ss_gamemaster', anonymous = True)
	gm = Game()
	gm.run()
