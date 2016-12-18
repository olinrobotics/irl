#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int16
from edwin.msg import *
import time
import subprocess
import alsaaudio, time, audioop

class EdwinAudioDetection():
	def __init__(self):
		time.sleep(1)

		rospy.init_node('edwin_audio_node', anonymous=True)
		self.pub = rospy.Publisher('edwin_sound', String, queue_size=10)
		#
		# rospy.Subscriber('/arm_status', Int16, self.arm_status_callback, queue_size=1)

		rospy.Subscriber('/all_control', String, self.control_callback, queue_size=10)
		self.listen = True


		self.mic = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NONBLOCK) #, device=plughw:2,0)
		# Set attributes: Mono, 8000 Hz, 16 bit little endian samples
		self.mic.setchannels(1)
		self.mic.setrate(8000)
		self.mic.setformat(alsaaudio.PCM_FORMAT_S16_LE)
		self.mic.setperiodsize(160)

		self.treshold = 0

	# def arm_status_callback(self, data):
	# 	print "RECEIVED ARM_STATUS: ", data.data
	# 	if data.data == 0:
	# 		self.arm_moving = False
	# 	elif data.data == 1:
	# 		self.arm_moving = True
	def control_callback(self, data):
		if "ed stop" in data.data:
			self.listen = False
		elif "ed go" in data.data:
			self.listen = True

	def calibrate(self):
		print 'Calibrating'
		timer = 0
		average_list = []
		while timer < 3.0:
			l,data = self.mic.read()
			test_sound = audioop.max(data, 2)
			if test_sound != 0:
				average_list.append(test_sound) #sampling sound in 2 second interval
			timer += .01

		if len(average_list) == 0:
			print 'Microphone is not working.  Retry'
			time.sleep(1)
			return False
		else:
			self.threshold = int(sum(average_list)/float(len(average_list))) #average volume.
		print "Threshold set as: ", self.threshold
		return True

	def run(self):
		absolute_threshold = 10030
		ret = self.calibrate()
		while ret == False:
			ret = self.calibrate()

		while not rospy.is_shutdown():
			if self.listen == False:
				continue

			if self.threshold > absolute_threshold:
				print 'Threshold too high.  Recalibrating'
				self.calibrate()

			soundbite = 0
			bite_length = 0
			peak_volume = 0

			cont = False
			while (cont == False) or peak_volume == 0:
				#only breaks out of loop when peak volume conditions are met
				l,data = self.mic.read()
				if l:
					level = audioop.max(data, 2)
					if level > self.threshold:
						if cont:
							soundbite += 1
							cont = False
						if level > peak_volume:
							peak_volume = level
					elif level < self.threshold: #Once hit under threshold, it will recognize next sound.
						cont = True
				time.sleep(.01)
				if peak_volume > 0:
					bite_length += .01
			print "BYTE LEN: ", bite_length
			self.pub.publish(str(bite_length) + ' ' + str(peak_volume))

if __name__ == '__main__':
	audio_eng = EdwinAudioDetection()
	audio_eng.run()
