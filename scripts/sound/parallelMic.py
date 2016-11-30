import threading
from multiprocessing import Process
import Queue
import pyaudio
import math
import matplotlib.pyplot
import numpy as np
import scipy
from scipy import signal
import struct


#Constants
CHUNK = 4096 #apparently 1024 causes overflow, anything above works.
CYCLE_LENGTH = .5 #How long each reading cycle is in seconds
RATE = 44100
SAMPLE_NUMBER = int(float(RATE)/float(CHUNK)*CYCLE_LENGTH) #Number of Samples
FORMAT = pyaudio.paFloat32

MIC_DISTANCE = .5 #Distance between Microphones in Meters.

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


	streamAnalysis()

	#clear for the next cycle
	streamData = [[], []]

def recordThread(mic_index):
	#The recording for each thread.
	for i in range(0,int(float(RATE)/float(CHUNK)*CYCLE_LENGTH)): # repeat until 1 second of recording, or how fast one wants to update.
		#This function reads one chunk of data for a stream to a corresponding array.
		data = micChannels[mic_index].read(CHUNK)
		streamData[mic_index].append(data)
		print "rec"
	#streamData[mic_index] = np.fromstring(np.asarray(streamData[mic_index]), 'Float32')
	streamData[mic_index] = np.asarray(streamData[mic_index])


#Stream analysis.  Should return an angle.  Does mathematics to compare
#the two different  streams to get said angle.
def streamAnalysis():
	channels = len(micChannelIndex)
	amp_threshold = 0.5
	time = []
	amplitude_data = []
	fft = []
	for j in range(0, channels): #convert to amplitude

		amplitude = np.fromstring(streamData[j], 'Float32')
		#low_values_indices = abs(amplitude) < amp_threshold
		#amplitude[low_values_indices] = 0
		time = np.arange(0, CYCLE_LENGTH, CYCLE_LENGTH/amplitude.size)

		amplitude_data.append(amplitude)

		#Fourier Fast Transform portion for filtering.
		#amplitude = np.append(amplitude, np.zeros(len(amplitude)))
		fft.append(abs(np.fft.fft(amplitude)))
		fft_freq = np.arange(-len(fft[j])/2, len(fft[j])/2, len(fft[j])/fft[j].size)
		#the FFT Scale still isn't quite right.

	for i in range(0, channels):
		matplotlib.pyplot.plot(time, amplitude_data[i])

	matplotlib.pyplot.show()

	for i in range(0, channels):
		matplotlib.pyplot.plot(fft_freq, fft[i])

	matplotlib.pyplot.show()

	if len(streamData) == 2: #Find Distance
		corrArray = np.correlate(amplitude_data[0], amplitude_data[1], "full")
		#Ideally would be valid, but ideal cross correlation means signals need to be identical.
		#Microphones aren't very omnidirectional?
		delay = corrArray[np.argmax(corrArray)] #Find the time delay, in seconds
		print delay

		ampMean0 = np.mean(np.absolute(amplitude_data[0]))
		ampMean1 = np.mean(np.absolute(amplitude_data[1]))

		if ampMean0/ampMean1 < ampMean1/ampMean0:
			ampRatio = ampMean0/ampMean1 #Amplitude Ratio.
		else:
			ampRatio = ampMean1/ampMean0

		print ampRatio
		b = ampRatio*delay*343/(1-ampRatio) #distance to close microphone.
		a = delay*343 + b #distance to further microphone.
		r = .5 * math.sqrt(4*b**2. + 4*a*b + 2*a**2. - MIC_DISTANCE**2.) #median
		print r
		#Currently the acos is out of theta because the calculated distance is pretty unreasonable.
		#theta = math.acos(r/MIC_DISTANCE + MIC_DISTANCE/(4*r) - b**2./(MIC_DISTANCE*r))

"""
Low pass filter, then amplify what is currently there to get a better cross correlation.
"""



def closeStreams():
	for i in range(0,len(micChannels)):
		micChannels[i].stop_stream()
		micChannels[i].close()
	p.terminate()

def equalize():
	#find the amplitude ratio between the different microphones.
	mic1 = np.asarray(streamData[0])
	mic2 = np.asarray(streamData[1])
	eqRatio = np.average(np.divide(mic1, mic2))

def calibrate():
	#Take a known sound, take a recording from a microphone, and figure out how to Transform
	#the recording to the known sound.

	#Will finish at some time, I guess now I'll just equalize.

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


	equalize()

	if len(streamData) == 2:
		streamData[1] = np.asArray(streamData[1])
		streamData[1] = np.toList(np.multiply(streamData[1], eqRatio))

	streamAnalysis()

def butter_filter(freq, fs=RATE, order=5):
	nyq = 0.5 * fs
	low = freq / nyq
	b, a = butter(order, low, 'lowpass')
	for i in streamData:
		i = np.asArray(i)
		i = lfilter(b, a, i)
		i = np.tolist(i)



if __name__ == '__main__':
	micStreamSetup() #Gets the mic indices and opens the streams
	calibrate()
	streamCycle()
	closeStreams()
