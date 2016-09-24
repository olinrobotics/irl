import threadingz
from multiprocessing import Process
import queue
import pyaudio
import math
import matplotlib

#Constants
CHUNK = 1024.
CYCLE_LENGTH = 1000. #How long each reading cycle is in milliseconds
FORMAT = pyaudio.paInt16
RATE = 44100.

#Some initialization things
p = pyaudio.PyAudio
micChannelIndex = [] #Array of the indices that the computer sees the microphone.  
micChannels = []
streamData = [] #Array of stream data.  There should be two entries, one for each string.
  



def findMicIndices():
	p = pyaudio.PyAudio()
	info = p.get_host_api_info_by_index(0)
	numdevices = info.get('deviceCount')
	#for each audio device, determine if is an input or an output and add it to the appropriate list and dictionary
	for i in range (0,numdevices):
		if p.get_device_info_by_host_api_device_index(0,i).get('maxInputChannels')>0:
			print "Input Device id ", i, " - ", p.get_device_info_by_host_api_device_index(0,i).get('name')
			micChannelIndex.append(i)

def openMic(computer_mic_index): #the index the computer sees the microphone as.  
	# opens up a new audio stream.  
	stream = p.open(format=FORMAT,
					channels=1,
					rate=RATE,
					input=True,
					frames_per_buffer=1024,
					input_device_index=computer_mic_index)
	
	micChannels.append(stream)
	
	micStream = []
	streamData.append(micStream)

def readStream(mic_index): #The index the user believes the microphone to be for convenience.  
		#This function reads one chunk of data for a stream to a corresponding array.
		data = micChannels(mic_index).read(CHUNK)
		streamData(mic_index).append(data)

#Cycles
def streamCycle():
	#Administrative thread that will generate the recording cycles
	
	threads = [] #Array of threads.
	
	#Initialize the mic streams
	for i in range(0,len(micChannels)):
		t = threading.Thread(target=recordThread, args=micChannelIndex(i))
		threads.append(t)
		#Run the threads in parallel
	for i in range(0,len(threads)):
		threads.start()
		#Streams should autoclose after completion.  


	streamAnalysis()	
	
	#clear for the next cycle
	streamData = [[] []]

def recordThread(mic_index):
	#The recording for each thread.  
	for i in range(0,int(RATE/CHUNK))# repeat until 1 second of recording, or how fast one wants to update. 
		readStream(mic_index)


#Stream analysis.  Should return an angle.  Does mathematics to compare
#the two different  streams to get said angle.   
def streamAnalysis():
	time = len(streamData(0))
	matplotlib.pyplot(time, streamData(0))
	matplotlib.pyplot(time, streamData(1))
	matplotlib.show()



if __name__ == '__main__':
