import rospy
import math
import st
import numpy as np
from std_msgs.msg import String
from edwin.msg import *
import time

class AudioObject:
    def __init__(self, name):
        self.name = name
        self.path = "../media" # TODO: fix this so it doesn't use '..'
        self.filename = "{}/{}.mp3".format(self.path, self.name)
        self.player = 'mplayer'
        self.playme = False

    def play_wave(self):
        """
        plays an inputted wav file
        """
        print self.filename
        cmd = '{} {}'.format(self.player, self.filename)
        #popen = subprocess.Popen([self.player, self.filename, "-ss", "30"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        popen = subprocess.Popen(cmd, shell=True)
        popen.communicate()
        return
        #popen.stdin.write("q")

    def update(self):
        if self.playme:
            time.sleep(1)
            print "playing sound file ", self.filename
            self.play_wave()
            self.playme = False
        return

class SoundBoard:
	def __init__(self):
        rospy.init_node('edwin_sounds', anonymous = True)
        rospy.Subscriber('/sound_cmd', Edwin_Sound, self.sound_callback, queue_size=10)
        self.behavior_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.sound_library = [] #Array of audio objects, to be added as needed.
        self.create_objects()

    def create_objects(self):
        pass

    def sound_callback(self, data):
    	call = data.msg #String indicating desired sound
		command = (x for x in self.sound_library if x.name == call) #Find in sound library
		command.play_wave()
		return

