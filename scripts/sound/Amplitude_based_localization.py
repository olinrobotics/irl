import threading
from multiprocessing import Process
import Queue
import pyaudio
import math
import matplotlib.pyplot
import numpy as np
import scipy
from scipy import signal
from scipy.signal import argrelextrema
from numpy import interp
import struct
import time

'''
The purpose of this script is to find a general direction from the relative amplitude
between the two microphones.
'''
class amplitude_localization:
	def __init__(self):
		#Constants
		self.CHUNK = 4096 #apparently 1024 causes overflow, anything above works.
		self.CYCLE_LENGTH = .125 #How long each reading cycle is in seconds
		self.RATE = 44100
		self.SAMPLE_NUMBER = int(float(self.RATE)/float(self.CHUNK)*self.CYCLE_LENGTH) #Number of Samples
		self.FORMAT = pyaudio.paFloat32

		self.MIC_DISTANCE = .4 #Distance between Microphones in Meters.
		self.MD = self.MIC_DISTANCE
		self.PERSON_DISTANCE = .25 #Distance of person to microphones
		self.PD = self.PERSON_DISTANCE

		#Some initialization things
		self.p = pyaudio.PyAudio()
		self.micChannelIndex = [] #list of the indices that the computer sees the microphone.
		self.micChannels = [] #List of the microphone streams.  Two entries.
		self.streamData = [] #List of stream data.  There should be two entries, one for each string.
		self.eqRatio = 1

	def micStreamSetup(self):
		p = pyaudio.PyAudio()
		info = p.get_host_api_info_by_index(0)
		numdevices = info.get('deviceCount')
		#for each audio device, determine if is an input or an output and add it to the appropriate list and dictionary
		for i in range(0,numdevices):
			if p.get_device_info_by_host_api_device_index(0,i).get('maxInputChannels')>0:
				print "Input Device id ", i, " - ", p.get_device_info_by_host_api_device_index(0,i).get('name')
				if p.get_device_info_by_host_api_device_index(0,i).get('name').find('USB') > -1: #USB Mics only
					self.micChannelIndex.append(i) #So far it looks like index 1 and 2.

		#print micChannelIndex #Verify that the mics are in the right places and lists.
		for i in self.micChannelIndex:
			self.openMic(int(i))

	def openMic(self, computer_mic_index): #the index the computer sees the microphone as.
		# opens up a new audio stream.

		stream = self.p.open(format=self.FORMAT,
						channels=1,
						rate=self.RATE,
						input=True,
						frames_per_buffer=self.CHUNK,
						input_device_index=computer_mic_index)

		self.micChannels.append(stream)

		micStreamData = []
		self.streamData.append(micStreamData)

	#Cycles
	def streamCycle(self):
		#Administrative thread that will generate the recording cycles

		threads = [] #Array of threads.

		#Initialize the mic streams
		for i in range(0,len(self.micChannels)):
			t = threading.Thread(target=self.recordThread, args=(i,))
			threads.append(t)
			#Run the threads in parallel
		for j in range(0,len(threads)):
			threads[j].start()
			#Streams should autoclose after completion.

		for k in range(0,len(threads)):
			threads[k].join()
			#Wait until all threads are finished.



	def recordThread(self, mic_index):

		#The recording for each thread.
		for i in range(0,int(float(self.RATE)/float(self.CHUNK)*self.CYCLE_LENGTH)): # repeat until 1 second of recording, or how fast one wants to update.
			#This function reads one chunk of data for a stream to a corresponding array.
			data = self.micChannels[mic_index].read(self.CHUNK)
			self.streamData[mic_index].append(data)
			print "rec"

		self.streamData[mic_index] = np.fromstring(np.asarray(self.streamData[mic_index]), 'Float32')

	def streamGraph(self):
		channels = len(self.micChannelIndex)
		amp_threshold = 0.5
		time = []
		amplitude_data = []


		for j in range(0, channels): #convert to amplitude

			amplitude = np.fromstring(self.streamData[j], 'Float32')

			#low_values_indices = abs(amplitude) < amp_threshold
			#amplitude[low_values_indices] = 0
			time = np.arange(0, self.CYCLE_LENGTH, self.CYCLE_LENGTH/amplitude.size)

			amplitude_data.append(amplitude)


		for i in range(0, channels):
			matplotlib.pyplot.plot(time, amplitude_data[i])

		matplotlib.pyplot.show()

		# for i in range(0, channels):
		# 	matplotlib.pyplot.plot(fft_freq, fft[i])
		#
		# matplotlib.pyplot.show()

		self.clearStreams()


	def closeStreams(self):
		for i in range(0,len(self.micChannels)):
			self.micChannels[i].stop_stream()
			self.micChannels[i].close()
		self.p.terminate()


	def average_amplitude(self): #Finds the average amplitudes

		averages = []
		for i in self.streamData:
			i = self.gauss_filter(i)
			peaks = argrelextrema(i, np.greater)
			stream_average = np.average(np.absolute(np.take(i, peaks)))
			averages.append(stream_average)

		return averages

	def getDirection(self):
		self.streamCycle()
		averages = self.average_amplitude()
		averages[0] = self.eqRatio * averages[0] #Equalize mic1
		ratio = averages[0]/averages[1]
		angle = interp(ratio, [self.MD/(self.PD+self.MD), (self.PD+self.MD)/self.MD], [0,180])

		return angle


	def calibrate(self):
		#Take a known sound, take a recording from a microphone, and figure out how to Transform
		#the recording to the known sound.
		self.eqRatio
		self.streamCycle()
		averages = self.average_amplitude()
		self.eqRatio = averages[1]/averages[0] #reciprocal of how much bigger mic 1 is than mic 2
		print averages
		print self.eqRatio
		self.streamGraph()

	def test(self):

		self.eqRatio
		print self.getDirection()
		self.streamData[0] = self.eqRatio * self.streamData[0]
		self.streamGraph()

	def clearStreams(self):
		self.streamData[0] = []
		self.streamData[1] = []

	def butter_filter_streams(self):
		for i in self.streamData:
			i = butter_lowpass_filter(i)


	def gauss_filter(self, data):
		window = signal.general_gaussian(51, p=0.5, sig=20)
	 	filtered = signal.fftconvolve(window, data)
	 	filtered = (np.average(data) / np.average(filtered)) * filtered
	 	filtered = np.roll(filtered, -25)
		return filtered

	"""
	Butter filter, should not need change, unless it's the parameters
	"""

	# def butter_lowpass_filter(self, data, cutOff=300, fs= self.RATE, order=4):
	#     nyq = 0.5 * self.RATE
	#     normalCutoff = cutOff / nyq
	#     b, a = scipy.signal.butter(order, normalCutoff, btype='low', analog = True)
	#     y = scipy.signal.lfilter(b, a, data)
	#     return y
	"""
	End Butter Filter
	"""




if __name__ == '__main__':
	a = amplitude_localization()
	a.micStreamSetup() #Gets the mic indices and opens the streams
	a.calibrate()
	a.test()

	print a.getDirection()
	a.clearStreams()

	#closeStreams()
