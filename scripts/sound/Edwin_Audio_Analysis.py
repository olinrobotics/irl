#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from edwin.msg import *
import time
import subprocess
import alsaaudio, time, audioop

#So the general idea is that you compare the two different input audios of the microphone
#volume, and then determine the angle to turn from there.  

class Edwin_Audio_Analysis():

	self.mic_right = alsaaudio.PCM(alsaaudio.PCM_CAPTURE,alsaaudio.PCM_NONBLOCK, device='default')
	self.mic_left = alsaaudio.PCM(alsaaudio.PCM_CAPTURE,alsaaudio.PCM_NONBLOCK, device='plughw:2,0')
	# Set attributes: Mono, 8000 Hz, 16 bit little endian samples, right mic
	self.mic_right.setchannels(1)
	self.mic_right.setrate(8000)
	self.mic_right.setformat(alsaaudio.PCM_FORMAT_S16_LE)
	self.mic_right.setperiodsize(160)
	# Set attributes: Mono, 8000 Hz, 16 bit little endian samples, left mic
	self.mic_left.setchannels(1)
	self.mic_left.setrate(8000)
	self.mic_left.setformat(alsaaudio.PCM_FORMAT_S16_LE)
	self.mic_left.setperiodsize(160)

	
	
	def compare(right_audio_volume, left_audio_volume): #returns direction
		difference = right_audio_volume - left_audio_volume
