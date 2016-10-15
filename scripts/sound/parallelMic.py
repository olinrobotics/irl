import threading
from multiprocessing import Process
import Queue
import pyaudio
import math
import matplotlib.pyplot
import numpy as np
import scipy


#Constants
CHUNK = 4096 #apparently 1024 causes overflow, anything above works.  
CYCLE_LENGTH = 1 #How long each reading cycle is in seconds
FORMAT = pyaudio.paFloat32
RATE = 44100
MIC_DISTANCE = .5 #Distance between Microphones in Meters.  

#Some initialization things
p = pyaudio.PyAudio()
micChannelIndex = [] #list of the indices that the computer sees the microphone.  
micChannels = [] #List of the microphone streams.  Two entries.
streamData = [] #List of stream data.  There should be two entries, one for each string.
  



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
	streamData[mic_index] = np.fromstring(np.asarray(streamData[mic_index]), np.int16)


#Stream analysis.  Should return an angle.  Does mathematics to compare
#the two different  streams to get said angle.   
def streamAnalysis():
	channels = len(micChannelIndex)
	time = []
	amplitude_data = []
	for j in range(0, channels): #convert to amplitude
		amplitude = streamData[j]
		time = np.arange(0, 1, 1./amplitude.size)
		amplitude_data.append(amplitude)
	
	for i in range(0, channels):
		matplotlib.pyplot.plot(time, amplitude_data[i])

	if len(streamData) == 2: #Find Distance
		corrArray = np.(streamData[0], streamData[1], "full")
		delay = numpy.argmax(corrArray) #Find the time delay, in seconds
		ampRatio = #Amplitude Ratio.  
		b = ampRatio*delay*343/(1-ampRatio) #distance to close microphone.  
		a = delay*343 + b #distance to further microphone.
		r = .5 * math.sqrt(4*b^2 + 4*a*b + 2*a^2 - MIC_DISTANCE^2) #median
		theta = math.acos(r/MIC_DISTANCE + MIC_DISTANCE/(4*r) - b^2/(MIC_DISTANCE*r))


	matplotlib.pyplot.show()

	if len(streamData) == 2:
		print "showing the difference"
		for i in range(0, channels):
			streamData[i] = scipy.fft(streamData[i])

		difference = np.subtract(streamData[0], streamData[1])
		average = np.mean(difference)
		print "The streams are" + str(average) + "%" + " different"

		ifft = scipy.ifft(streamData[0]*scipy.conj(streamData[1]))
		time_shift = argmax(abs(ifft))
		print time_shift


def closeStreams():
	for i in range(0,len(micChannels)):
		micChannels[i].stop_stream()
		micChannels[i].close()
	p.terminate()


if __name__ == '__main__':
	micStreamSetup() #Gets the mic indices and opens the streams
	streamCycle()
	closeStreams()
