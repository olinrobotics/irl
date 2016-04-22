import numpy as np
from scipy.io import wavfile
import wave

class SoundManipulator:
    def __init__(self, filename):
        self.filename = filename
    
    #def speedx(self, data, factor):
     #   """ Multiplies the sound's speed by some `factor` """
      #  sound_array = data
        #if f_in <= 0:
        #    print "Please use valid factor"
        #    return
        #factor = 1./f_in
        #print "FACTOR: ", factor
        
        #indices = np.round( np.arange(0, len(sound_array), factor) ) #creates a list of indices in the
        #Sound array to either skip or duplicate.  
        #indices = indices[indices < len(sound_array)].astype(int) #Takes those indices and then 
        #cast all of them less than the length of the sound array as int.  
        #return sound_array[ indices.astype(int) ]

    def time_accel(self, filename, factor): #The Emiya Family Magic now as a code function! jk.
        CHANNELS = 1
        swidth = 2
        Change_RATE = factor

        spf = wave.open(filename, 'rb')
        RATE=spf.getframerate()
        signal = spf.readframes(-1)

        wf = wave.open("alter", 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(swidth)
        wf.setframerate(RATE*Change_RATE)
        wf.writeframes(signal)
        wf.close()

    def stretch(self, filename, factor, window_size=2048, h=512):
        """ Stretches the sound by a factor """
        fs, data = wavfile.read(filename)
        sound_array = data

        phase  = np.zeros(window_size)
        hanning_window = np.hanning(window_size)
        result = np.zeros( len(sound_array) /factor + window_size)

        for i in np.arange(0, len(sound_array)-(window_size+h), h*factor):

            # two potentially overlapping subarrays
            a1 = sound_array[i: i + window_size]
            a2 = sound_array[i + h: i + window_size + h]

            # resynchronize the second array on the first
            s1 =  np.fft.fft(hanning_window * a1)
            s2 =  np.fft.fft(hanning_window * a2)
            phase = (phase + np.angle(s2/s1)) % 2*np.pi
            #a2_rephased = np.fft.ifft(np.abs(s2)*np.exp(1j*phase))
            a2_rephased = np.fft.ifft(np.abs(s2)*np.exp(1j*phase))

            # add to result
            i2 = int(i/factor)
            result[i2 : i2 + window_size] += hanning_window*a2_rephased

        result = ((2**(16-4)) * result/result.max()) # normalize (16bit)
        result = result.astype('int16')
        
        wavfile.write('pitchshift.wav', len(result), result) #write


    def stretch_alternate(self, filename, factor, window_size=2048, h=512):
		fs, data = wavfile.read(filename)
		#Conversions for the method copy
		signalin = data
		N = window_size
		H = N/4
		L = len(signalin)
		tscale = 1.0/factor

		# signal blocks for processing and output
		phi  = np.zeros(N)
		out = np.zeros(N, dtype=complex)
		sigout = np.zeros(L/tscale+N)

		# max input amp, window
		amp = max(signalin)
		win = np.hanning(N)
		p = 0
		pp = 0

		while p < L-(N+H):

			# take the spectra of two consecutive windows
			p1 = int(p)
			spec1 =  np.fft.fft(win*signalin[p1:p1+N])
			spec2 =  np.fft.fft(win*signalin[p1+H:p1+N+H])
			# take their phase difference and integrate
			phi += (np.angle(spec2) - np.angle(spec1))

			# bring the phase back to between pi and -pi
			while (phi < -np.pi): 
				phi += 2*np.pi
			while (phi >= np.pi): 
				phi -= 2*np.pi
			out.real, out.imag = cos(phi), sin(phi)
			# inverse FFT and overlap-add
			sigout[pp:pp+N] += win*np.fft.ifft(abs(spec2)*out)
			pp += H
			p += H*tscale

		wavfile.write("pitchshift",sr,array(amp*sigout/max(sigout), dtype='int16'))

    def pitchshift(self, n, filename, window_size=2**13, h=2**11):
        """ Changes the pitch of a sound by ``n`` semitones. """
        factor = 2**(1.0 * n / 12.0)
        self.time_accel(filename, factor) #Write it.
        #stretched = self.stretch_alternate("alter", 1.0/factor, window_size, h)
        #self.volumeshift("alter", 1.0)

    def volumeshift(self, filename, factor):
    	#Increases or decreases the volume of the array.
    	factor = 10**(factor/20.0)
    	fs, data = wavfile.read(filename)
    	data = np.multiply(data, factor)
    	print factor
    	wavfile.write("volume_shift", fs, data)

    def load(self, sound_file):
    	w = wave.open(sound_file, "rb")

    def play(self, filename):
        fs, data = wavfile.read(filename)
        #scaled = np.int16(data/np.max(np.abs(data)) * 32767)

        wavfile.write('test.wav', fs, data)


        #spd2 = self.stretch(data, 5)
        #wavfile.write('spd2.wav', len(spd2), spd2)

if __name__ == '__main__':
	# sound = load("r2d2.wav")
    #testfile = wave.open("./media/sad.wav", 'r')
    #print testfile.getnchannels()
    sound = SoundManipulator("./media/sad.wav")
    #sound.time_accel(sound.filename, .5)
    sound.pitchshift(12, sound.filename)
    #sound.play(sound.filename)
