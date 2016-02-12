#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from edwin.msg import *
import time
import subprocess





class AudioObject:
	def __init__(self, name):
		self.name = name
		self.path = "media"
		self.filename = "{}/{}.wav".format(self.path, self.name)
		self.player = 'mplayer'

	def play_wave(self):
		"""
		plays an inputted wav file
		"""
		print self.filename
		cmd = '{} {}'.format(self.player, self.filename)
		#popen = subprocess.Popen([self.player, self.filename, "-ss", "30"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

		popen = subprocess.Popen(cmd, shell=True)
		popen.communicate()
		return 1
		#popenj.stdin.write("q")

class SoundBoard:
	def __init__(self):
		#self.Sound_pub = Rospy.Publisher('behaviors_cmd/sound_cmd', String, queue_size=10)
		rospy.init_node('edwin_sounds', anonymous = True)
		rospy.Subscriber('/sound_cmd', String, self.sound_callback)
		self.sound_library =  self.create_objects()

	def create_objects(self): #Reads all the files in media, instantiates them as audio_objects
		AudioList = []
		AudioList.append(AudioObject("Battlecry")) #Example/test file
		AudioList.append(AudioObject("Dinosaur1"))
		AudioList.append(AudioObject("Dinosaur_groan"))
		AudioList.append(AudioObject("Dinosaur_Roar1"))
		AudioList.append(AudioObject("Dinosaur_Roar2"))
		AudioList.append(AudioObject("Dinosaur_snort"))
		AudioList.append(AudioObject("Dragon"))
		AudioList.append(AudioObject("Falcon"))
		
		return AudioList

	def sound_callback(self, data):
		call =  data.data #String indicating desired sound
		command = next((x for x in self.sound_library if x.name == call)) #Find in sound library
		print "playing" + data.data
		command.play_wave()
		return 1

if __name__ == '__main__':
	sound = SoundBoard()
	rospy.spin()
