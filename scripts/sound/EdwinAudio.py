import alsaaudio, time, audioop

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
calibrate = False
timer = 0
average_list = []
thresh = 450

while True:
# Read data from device
	l,data = mic.read()
	
	if calibrate == False:
		average_list.append(audioop.max(data, 2)) #sampling sound in 10 second intervals
	if (timer > 2.0) and (calibrate == False):
		calibrate = True
		thresh = int(sum(average_list)/float(len(average_list))) #average sound.  
		print thresh, "ready"
		average_list = []
	elif l and calibrate:
		level = audioop.max(data, 2)
		if level > thresh:
				print "too loud"


	time.sleep(.001)
	timer += .001