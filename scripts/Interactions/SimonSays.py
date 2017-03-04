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

"""
This class plays the game of Simon Says in three different ways: by himself, as Simon with a  player,
or as a player with Simon. The basic game is that Simon will tell the player to perform a certain action,
and the player must only perform a movement when Simon adds 'simon says' to the beginning of the sentence.
The player loses if he makes an incorrect move or moves under a command that didn't have the 'simon says' prefix.

The three modes are split up and draw from a parent class that holds general game functionality. The basic three steps in the game for Edwin
regardless of his role is:

1. listen/say a command
2. act/check on response to command
3. continue until a set number of turns or until the player loses
"""

SIMON_NAME = "simon"
PLAYER_NAME = "player"

class SimonSays:
	"""
	The master class of the game Simon Says, which holds all the general actions of the Game
	To be the parent of the actual game modes:
	1. AutonomousSimon - plays by himself
	2. SimonEdwin - Edwin is the Simon and issues commands to the user
	3. SimonUser - The user if the Simon and tell Edwin what to do
	"""
	def __init__(self, max_turns = 5):
		#init ROS nodes
		rospy.init_node('ss_gamemaster', anonymous = True)

		#ROS subscriber variables
		self.ready_to_listen = False
		self.heard_cmd = None
		self.body_points = None
		self.frame = None

		#Edwin's current command of interest
		self.current_cmd = None

		#max iterations per game, and variable to keep track of game procession
		self.max_turns = max_turns
		self.no_mistakes = True  #TODO: this must be incorporated into the code

		#the dictionaries that contain what Edwin will do based on a Simon command
		self.command_2_speech = {}
		self.command_2_motion = {}

		#populate the two dictionaries for when Edwin is Simon and when he is the player
		self.populate_simon_dictionaries()
		self.populate_player_dictionaries()

		#for the image
		self.bridge = CvBridge()

		#init ROS publishers for actuation and stated speech
		self.behavior_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
		self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
		self.behav_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
		self.say_pub = rospy.Publisher('say_cmd', String, queue_size = 1)

		#init ROS subscribers to camera, heard speech, and skeleton
		self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)
		self.hear_sub = rospy.Subscriber("decoded_speech", String, self.hear_callback)
		self.skelesub = rospy.Subsriber("skeleton", Bones, self.skeleton_callback)


	def img_callback(self, data):
		try:
			self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

	def hear_callback(self, speech):
		if self.ready_to_listen:
			self.heard_cmd = speech.data

	def skeleton_callback(self, skeleton):
		"""
        Parses out skeleton data

		Skeleton is stored as an object with 15 different variables, one for each skeleton point

		h - head
		n - neck
		t - torso

		rs - right shoulder
		re - right elbow
		rh - right hand

		rp - right hip
		rk - right knee
		rf - right foot

		ls - left shoulder
		le - left elbow
		lh - left hand

		lp - left hip
		lk - left knee
		lf - left foot
        """

		self.body_points = skeleton



	def populate_simon_dictionaries(self):
		"""
		Fill up the Simon command dictionary possible commands
		"""
		self.command_2_speech["turn_around"] = "turn around"
		self.command_2_speech["touch_head"] = "touch your head"
		self.command_2_speech["rub_tummy"] = "rub your tummy"
		self.command_2_speech["high5_self"] = "give yourself a high five"
		self.command_2_speech["hug_self"] = "hug yourself"
		self.command_2_speech["clap_hands"] = "clap your hands above your head"
		self.command_2_speech["flex_muscles"] = "flex your muscles"
		self.command_2_speech["hands_up"] = "put your hands way up"
		self.command_2_speech["disco"] = "do the disco"
		self.command_2_speech["bow"] = "bow down to the master"


	def populate_player_dictionaries(self):
		"""
		Fill up the Player command dictionary possible commands
		"""
		#TODO: make these commands into actual behaviors
		self.command_2_speech["turn_around"] = ["turn_around"]
		self.command_2_speech["touch_head"] = ["touch_head"]
		self.command_2_speech["rub_tummy"] = ['rub_tummy']
		self.command_2_speech["high5_self"] = ['high_five']
		self.command_2_speech["hug_self"] = ["hug_self"]
		self.command_2_speech["clap_hands"] = ["clap_hands"]
		self.command_2_speech["flex_muscles"] = ["flex_muscles"]
		self.command_2_speech["hands_up"] = ["sky_reach"]
		self.command_2_speech["disco"] = ["disco"]
		self.command_2_speech["bow"] = ["bow"]



	def check_stillness(self):
		"""
		If Edwin is Simon:

		This function is a helper function to the check_simon_response method
		It determines whether the user has moved based on the sliding window
		"""
		#TODO
		pass




	def simon_say_command(self):
		"""
		If Edwin is Simon:

		This function issues a Simon command.
		It is said outloud, so depends on the tts_engine to be running
		"""
		command = random.choice(self.command_dictionary.keys())
		self.current_cmd = random.choice(["simon says, ", ""]) + self.command_dictionary[command]
		print self.current_cmd
		self.say_pub.publish(self.current_cmd)


	def simon_check_response(self):
		"""
		If Edwin is Simon:

		This function checks to see if the players have followed Edwin's cmd
		This relies on skeleton tracker to be functional
		"""
		#TODO: finish this implementation, what happens with a simon says, and what happens when checking for stillness

		if "simon says" in self.current_cmd:
			print "checking for the simon command"
			#will check with Katya and Yichen's module for the correct gesture
			pass
		else:
			self.check_stillness()


	def player_listen_for_simon(self):
		"""
		If Edwin is the player:
		This function listens to a spoken Simon command from user

		This relies on stt_engine to be running
		"""
		#TODO: check this implementation
		self.ready_to_listen = True

		#do nothing while waiting for the speech command
		while not self.heard_cmd:
			continue

		if "simon says" in self.heard_cmd:
			cmd = self.heard_cmd.substitute("simon says ", "")
			motions = self.command_2_motion.get(cmd)

			if motions:
				self.current_cmd = cmd
				print "GOT CMD: ", cmd
		else:
			self.current_cmd = None

		#if simon says isn't in the command, than Edwin does nothing
		self.ready_to_listen = False

	def player_follow_simon_cmd(self):
		"""
		If Edwin is the player:

		This function takes the command from listen_for_simon() and interprets
		The resulting command is sent to arm_node for physical motion
		"""
		#TODO: finish this implementation

		if self.current_cmd:
			motions = self.command_2_motion.get(cmd, None)
			if motions:
				for m in motions:
					print "GOT MOTION: ", m
					self.behavior_pub.publish(m)

	def run(self):
		pass



class AutonomousSimon(SimonSays):
	"""
	Game Mode when Edwin plays against himself, meaning:
 	1. He will say a Simon command
	2. He will then move based on his own command
	"""
	def __init__(self):
		super(AutonomousSimon, self).__init__()


	def run(self):
		time.sleep(2)
		print "playing by myself"

		while self.max_turns > 0:
			self.max_turns -= 1

			self.simon_say_command()
			self.player_follow_simon_cmd()


		print "Finished playing by myself, hope you enjoyed the demo!"




class EdwinSimon(SimonSays):
	"""
	Game mode where Edwin is Simon, meaning:
	1. Edwin will say a Simon command
	2. Edwin will check if the user has performed the command
	3. Game continues for 5 turns or until user fails to perform command
	"""
	def __init__(self):
		super(EdwinSimon, self).__init__()


	def run(self):

		time.sleep(2)
		print "playing as Simon"

		while self.max_turns > 0 and self.no_mistakes:
			self.max_turns -= 1

			self.simon_say_command()
			self.simon_check_response()

		print "Finished playing with player, hope you enjoyed!"


class EdwinPlayer(SimonSays):
	"""
	Game mode where Edwin is the player, meaning:
	1. Edwin will listen for a Simon command from the user
	2. Edwin will perform the command based on what he heard
	3. Game continues until the user says Edwin is wrong, or until 5 turns
	"""
	def __init__(self):
		super(EdwinPlayer, self).__init__()

	def run(self):
		time.sleep(2)
		print "playing as Player"

		while self.max_turns > 0 and self.no_mistakes:
			self.max_turns -= 1

			self.player_listen_for_simon()
			self.player_follow_simon_cmd()

		print "Finished playing with Simon, hope you enjoyed!"

"""
Things I need to do for this code
1. finish up implementations in the runs, the simon and player methods
2. add in the abilty to lose
3. finish up code arch
4. write in behaviors to simon commands
"""


if __name__ == '__main__':
	mode = raw_input("Is Edwin the simon or player?")

	if mode == PLAYER_NAME:
		game = EdwinPlayer()
	elif mode == SIMON_NAME:
		game = EdwinSimon()
	else:
		game = AutonomousSimon()


	game.run()
