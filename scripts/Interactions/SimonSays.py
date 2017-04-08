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
from edwin.msg import Edwin_Shape, Bones
from cv_bridge import CvBridge, CvBridgeError

EDWIN_NAME = "edwin"
USER_NAME = "user"

class Game:
	def __init__(self, max_turns = 5):
		self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
		self.behav_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
		self.say_pub = rospy.Publisher('say_cmd', String, queue_size = 1)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)
		self.hear_sub = rospy.Subscriber("decoded_speech", String, self.hear_callback)
		#self.skelesub = rospy.Subscriber("skeleton", Bones, self.skeleton_callback)
		self.gesturesub = rospy.Subscriber("skeleton_detect", String, self.gest_callback)

		self.current_cmd = None
		self.heard_cmd = None
		self.ready_to_listen = False

		self.max_turns = max_turns
		self.command_2_speech = {}
		self.command_2_motion = {}
		self.command_2_rules = {}
		self.command_dictionary = {}

		self.populate_dictionaries()
		self.populate_command_dictionaries()

		self.simonless_gest = None

	def img_callback(self, data):
		try:
			self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print e


	def hear_callback(self, data):
		if self.ready_to_listen:
			self.heard_cmd = data.data

	def gest_callback(self,data):
		self.gesture = data



	def populate_dictionaries(self):
		"""Fill up the Simon command dictionary possible command"""
		self.command_2_speech["turn_around"] = "turn around"
		self.command_2_motion["turn_around"] = ["R_ttt"]
		self.command_2_rules["turn_around"] = [(1,2), (2,0)]

		self.command_2_speech["touch_head"] = "touch your head"
		self.command_2_speech["rub_tummy"] = "rub your tummy"
		self.command_2_speech["high5_self"] = "give yourself a high five"
		self.command_2_speech["hug_self"] = "hug yourself"
		self.command_2_speech["breakdance"] = "breakdance!"
		self.command_2_speech["clap_hands"] = "clap your hands!"

	def populate_command_dictionaries(self):
		"""Fill the command dictionary"""
		self.command_dictionary["touch_head"] = "Touch your head with left hand"
		self.command_dictionary["rub_tummy"] = "Rub your tummy with both hands"
		self.command_dictionary["high_five"] = "High five to your left"
		self.command_dictionary["wave"] = "Wave to your right"
		self.command_dictionary["hug"] = "Hug yourself"
		self.command_dictionary["dab"] = "Dab into your right elbow"
		self.command_dictionary["disco"] = "Disco with your right hand"
		self.command_dictionary["bow"] = "Just Bow"
		self.command_dictionary["star"] = "Do a starfish"
		self.command_dictionary["heart"] = "Form a heart with your arms"

	def field_scan(self):
		time.sleep(5)

		cv2.imshow("img", self.frame)
		c = cv2.waitKey(1)

	def issue_simon_cmd(self):
		"""If Simon is Edwin:

		This function issues a Simon command.
		It is said outloud, so depends on the tts_engine to be running"""

		command = random.choice(self.command_dictionary.keys())
		self.current_cmd = random.choice(["simon says, ", ""]) + command
		self.say_pub.publish(self.current_cmd)

	def check_simon_response(self):
		"""If Simon is Edwin:

		This function checks to see if the players have followed Edwin's cmd
		This relies on skeleton tracker to be functional
		"""

		if "simon says" in self.current_cmd:
			command = self.current_cmd.substitute("simon says, ","")
			command_gest  = self.command_dictionary.get(command)
			if command_gest  == self.gesture:
				print('Good job!')
				self.simonless_gest = self.gesture
			else:
				print('Try again!')
		else:
			if self.simonless_gest == self.gesture:
				print('Good job!')
			else:
				print('Try again!')
			#check that kids aren't moving

		#TODO: add behavior for success / failure of following command

	def listen_for_simon(self):
		"""If Simon is the User:
		This function listens to a spoken Simon command from user

		This relies on stt_engine to be running"""
		self.ready_to_listen = True

		#do nothing while waiting for the speech command
		while not self.heard_cmd:
			continue

		if "simon says" in self.heard_cmd:
			cmd = self.heard_cmd.substitute("simon says ", "")
			motions = self.command_2_motion.get(cmd, None)

			if motions:
				self.current_cmd = cmd
				print "GOT CMD: ", cmd
		else:
			self.current_cmd = None

		#if simon says isn't in the command, than Edwin does nothing
		self.ready_to_listen = False

	def follow_simon_cmd(self):
		"""If Simon is the User:

		This function takes the command from listen_for_simon() and interprets
		The resulting command is sent to arm_node for physical motion"""

		if self.current_cmd:
			motions = self.command_2_motion.get(cmd, None)
			if motions:
				for m in motions:
					print "GOT MOTION: ", m
					#TODO: add publish to arm_cmd for edwin to actually move

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
	while True:
		raw_input("enter to go to next")
		gm.issue_simon_cmd()
	# gm.run()
