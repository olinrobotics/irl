#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from edwin.msg import *
import time
import subprocess
import alsaaudio, time, audioop

#Edwin Audio basically detects and publishes the peak volume and length of a soundbite.  

class EdwinAudioDetection():
	def __init__(self):
			# Open the device in nonblocking capture mode. The last argument could
		# just as well have been zero for blocking mode. Then we could have
		# left out the sleep call in the bottom of the loop
		mic = alsaaudio.PCM(alsaaudio.PCM_CAPTURE,alsaaudio.PCM_NONBLOCK)

		# Set attributes: Mono, 8000 Hz, 16 bit little endian samples
		mic.setchannels(1)
		mic.setrate(8000)
		mic.setformat(alsaaudio.PCM_FORMAT_S16_LE)
		mic.setperiodsize(160)
		# The period size controls the internal number of frames per period.
		# The significance of this parameter is documented in the ALSA api.
		# For our purposes, it is suficcient to know that reads from the device
		# will return this many frames. Each frame being 2 bytes long.
		# This means that the reads below will return either 320 bytes of data
		# or 0 bytes of data. The latter is possible because we are in nonblocking
		# mode.
		pub = rospy.Publisher('Edwin_Sound_Detection', String, queue_size=10)
		rospy.init_node('Edwin_Sound_Detector', anonymous=True)
		#rospy.Subscriber('Edwin_Sound_Detection_Runner', String, self.run)
		#The subscriber tells whether or not to run the Run script, which listens to one sound
		#And then as of now, returns the length of the soundbite.  
	#def test_publish():
	#	test_pub = rospy.Publisher('Edwin_Sound_Detection_Runner', String, queue_size=10)
	#	test_pub.publish('run') #TestScript that gives the command to run the audio length

	def calibrate():
		print 'calibrating'
		timer = 0
		average_list = []
		while timer < 3.0:
			l,data = mic.read()
			test_sound = audioop.max(data, 2)
			if test_sound != 0:
				average_list.append(test_sound) #sampling sound in 2 second interval
			timer += .01

		thresh = int(sum(average_list)/float(len(average_list))) #average volume.  

		print thresh, "ready"
		return thresh

	def run(self, data):
		absolute_threshold = 2030 #If above this level, then will recalibrate

		if data.data == 'run': #If Edwin wants to listen
			threshold = calibrate()
			if threshold > absolute_threshold:
				print 'Threshold too high.  Recalibrating'
				threshold = calibrate()
			soundbite = 0 #The number of the soundbite.
			bite_length = 0 #How long the soundbite is
			peak_volume = 0 #Self explanatory.
			cont = False #So continue
			while (cont == False) or peak_volume == 0: #make sure something said
				l,data = mic.read() # Read data from device
				if l:
					level = audioop.max(data, 2)
					if level > threshold:
						print "Listening Session:", str(soundbite)
						if cont:
							soundbite += 1
							cont = False
						if level > peak_volume:
							peak_volume = level
					elif level < threshold: #Once hit under threshold, it will recognize next sound.
						cont = True

				time.sleep(.01)
				if peak_volume > 0:
					bite_length += .01
			pub.publish(str(bite_length) + ' ' + str(peak_volume)) #Maybe shouldn't be a Tuple.
			return True

if __name__ == '__main__':
	#test_publish()
	rospy.spin()