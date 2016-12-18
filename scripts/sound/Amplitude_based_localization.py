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

'''
The purpose of this script is to find a general direction from the relative amplitude
between the two microphones.
'''

#Constants
CHUNK = 4096 #apparently 1024 causes overflow, anything above works.
CYCLE_LENGTH = .125 #How long each reading cycle is in seconds
RATE = 44100
SAMPLE_NUMBER = int(float(RATE)/float(CHUNK)*CYCLE_LENGTH) #Number of Samples
FORMAT = pyaudio.paFloat32

MIC_DISTANCE = .4 #Distance between Microphones in Meters.
MD = MIC_DISTANCE
PERSON_DISTANCE = .25 #Distance of person to microphones
PD = PERSON_DISTANCE

#Some initialization things
p = pyaudio.PyAudio()
micChannelIndex = [] #list of the indices that the computer sees the microphone.
micChannels = [] #List of the microphone streams.  Two entries.
streamData = [] #List of stream data.  There should be two entries, one for each string.
eqRatio = 1

def micStreamSetup():
	p = pyaudio.PyAudio()
	info = p.get_host_api_info_by_index(0)
	numdevices = info.get('deviceCount')
	#for each audio device, determine if is an input or an output and add it to the appropriate list and dictionary
	for i in range(0,numdevices):
		if p.get_device_info_by_host_api_device_index(0,i).get('maxInputChannels')>0:
			print "Input Device id ", i, " - ", p.get_device_info_by_host_api_device_index(0,i).get('name')
			if p.get_device_info_by_host_api_device_index(0,i).get('name').find('USB') > -1: #USB Mics only
				micChannelIndex.append(i) #So far it looks like index 1 and 2.

	#print micChannelIndex #Verify that the mics are in the right places and lists.
	for i in micChannelIndex:
		openMic(int(i))

def openMic(computer_mic_index): #the index the computer sees the microphone as.
	# opens up a new audio stream.

	stream = p.open(format=FORMAT,
					channels=1,
					rate=RATE,
					input=True,
					frames_per_buffer=CHUNK,
					input_device_index=computer_mic_index)

	micChannels.append(stream)

	micStreamData = []
	streamData.append(micStreamData)

#Cycles
def streamCycle():
	#Administrative thread that will generate the recording cycles

	threads = [] #Array of threads.

	#Initialize the mic streams
	for i in range(0,len(micChannels)):
		t = threading.Thread(target=recordThread, args=(i,))
		threads.append(t)
		#Run the threads in parallel
	for j in range(0,len(threads)):
		threads[j].start()
		#Streams should autoclose after completion.

	for k in range(0,len(threads)):
		threads[k].join()
		#Wait until all threads are finished.



def recordThread(mic_index):
	global streamData
	#The recording for each thread.
	for i in range(0,int(float(RATE)/float(CHUNK)*CYCLE_LENGTH)): # repeat until 1 second of recording, or how fast one wants to update.
		#This function reads one chunk of data for a stream to a corresponding array.
		data = micChannels[mic_index].read(CHUNK)
		streamData[mic_index].append(data)
		print "rec"

	streamData[mic_index] = np.fromstring(np.asarray(streamData[mic_index]), 'Float32')


	#streamData[mic_index] = np.asarray(streamData[mic_index])


#Stream analysis.  Should return an angle.  Does mathematics to compare
#the two different  streams to get said angle.
def streamGraph():
	channels = len(micChannelIndex)
	amp_threshold = 0.5
	time = []
	amplitude_data = []
    #fft[]
	global streamData

	for j in range(0, channels): #convert to amplitude

		amplitude = np.fromstring(streamData[j], 'Float32')

		#low_values_indices = abs(amplitude) < amp_threshold
		#amplitude[low_values_indices] = 0
		time = np.arange(0, CYCLE_LENGTH, CYCLE_LENGTH/amplitude.size)

		amplitude_data.append(amplitude)

		#Fourier Fast Transform portion for filtering.
		#amplitude = np.append(amplitude, np.zeros(len(amplitude)))
		#fft.append(abs(np.fft.fft(amplitude)))
		#fft_freq = np.arange(-len(fft[j])/2, len(fft[j])/2, len(fft[j])/fft[j].size)
		#the FFT Scale still isn't quite right.

	for i in range(0, channels):
		matplotlib.pyplot.plot(time, amplitude_data[i])

	matplotlib.pyplot.show()

	# for i in range(0, channels):
	# 	matplotlib.pyplot.plot(fft_freq, fft[i])
	#
	# matplotlib.pyplot.show()

	streamData[0] = []
	streamData[1] = []

def streamAnalysis():
	equalize()


def closeStreams():
	for i in range(0,len(micChannels)):
		micChannels[i].stop_stream()
		micChannels[i].close()
	p.terminate()


def average_amplitude(): #Finds the average amplitudes
	global streamData
	averages = []
	for i in streamData:

		i = gauss_filter(i)
		peaks = argrelextrema(i, np.greater)
		stream_average = np.average(np.absolute(np.take(i, peaks)))
		averages.append(stream_average)
	print averages
	return averages

def getDirection():
	averages = average_amplitude()
	averages[0] = eqRatio * averages[0] #Equalize mic1
	ratio = averages[0]/averages[1]
	angle = interp(ratio, [MD/(PD+MD), (PD+MD)/MD], [0,180])
	return angle




def calibrate():
	#Take a known sound, take a recording from a microphone, and figure out how to Transform
	#the recording to the known sound.
	global eqRatio
	streamCycle()
	averages = average_amplitude()
	eqRatio = averages[1]/averages[0] #reciprocal of how much bigger mic 1 is than mic 2
	print eqRatio
	streamGraph()

def test():
	streamCycle()
	getDirection()
	streamGraph()


def butter_filter_streams():
	for i in streamData:
		i = butter_lowpass_filter(i)


def gauss_filter(data):
	window = signal.general_gaussian(51, p=0.5, sig=20)
 	filtered = signal.fftconvolve(window, data)
 	filtered = (np.average(data) / np.average(filtered)) * filtered
 	filtered = np.roll(filtered, -25)
	return filtered

"""
Butter filter, should not need change, unless it's the parameters
"""

def butter_lowpass_filter(data, cutOff=300, fs=RATE, order=4):
    nyq = 0.5 * RATE
    normalCutoff = cutOff / nyq
    b, a = scipy.signal.butter(order, normalCutoff, btype='low', analog = True)
    y = scipy.signal.lfilter(b, a, data)
    return y
"""
End Butter Filter
"""




if __name__ == '__main__':
	micStreamSetup() #Gets the mic indices and opens the streams
	calibrate()
	test()
	closeStreams()
