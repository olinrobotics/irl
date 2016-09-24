import threading
import queue
import pyaudio
import math

#Constants
CHUNK = 1024


#Some initialization things
p = pyaudio.PyAudio
micChannelIndex = [0 1] #Array of the indices that the computer sees the microphone.  
micChannels = []
streamData = [] #Array of stream data.  There should be two entries, one for each string.
record = False




def openMic(computer_mic_index): #the index the computer sees the microphone as
	stream = p.open(format=pyaudio.paInt16,
					channels=1,
					rate=44100,
					input=True,
					frames_per_buffer=1024,
					input_device_index=computer_mic_index)
	
	micChannels.append(stream)
	
	micStream = []
	streamData.append(micStream)

def readStream(mic_index): #The index the user believes the microphone to be for convenience.  
	while record = True:
		data = micChannels(mic_index).read(CHUNK)
		streamData(mic_index).append(data)

for i in range(len(micChannels)):
	openMic(micChannelIndex(i))
	t = threading.Thread(target=readStream, args=micChannelIndex(i))
	micChannels.appeand(t)

for i in range(len(micChannels)):




