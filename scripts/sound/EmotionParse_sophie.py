import numpy as np
from scipy.io import wavfile

class SoundManipulator:
    def __init__(self):
        self.soundfile = ""

    def speedx(self, f_in):
        """ Multiplies the sound's speed by some `factor` """
        if f_in <= 0:
            print "Please use valid factor"
            return
        factor = 1./f_in
        print "FACTOR: ", factor
        indices = np.round( np.arange(0, len(sound_array), factor) )
        indices = indices[indices < len(sound_array)].astype(int)
        return sound_array[ indices.astype(int) ]

    def stretch(self, f, window_size, h):
        """ Stretches the sound by a factor `f` """

        phase  = np.zeros(window_size)
        hanning_window = np.hanning(window_size)
        result = np.zeros( len(sound_array) /f + window_size)

        for i in np.arange(0, len(sound_array)-(window_size+h), h*f):

            # two potentially overlapping subarrays
            a1 = sound_array[i: i + window_size]
            a2 = sound_array[i + h: i + window_size + h]

            # resynchronize the second array on the first
            s1 =  np.fft.fft(hanning_window * a1)
            s2 =  np.fft.fft(hanning_window * a2)
            phase = (phase + np.angle(s2/s1)) % 2*np.pi
            a2_rephased = np.fft.ifft(np.abs(s2)*np.exp(1j*phase))

            # add to result
            i2 = int(i/f)
            result[i2 : i2 + window_size] += hanning_window*a2_rephased

        result = ((2**(16-4)) * result/result.max()) # normalize (16bit)

        return result.astype('int16')

    def pitchshift(self, n, window_size=2**13, h=2**11):
        """ Changes the pitch of a sound by ``n`` semitones. """
        factor = 2**(1.0 * n / 12.0)
        stretched = stretch(snd_array, 1.0/factor, window_size, h)
        return speedx(stretched[window_size:], factor)

    def volumeshift(self, factor):
    	#Increases or decreases the volume of the array.
    	pass

    def load(self, sound_file):
    	w = wave.open(sound_file, "rb")

    def play(self, filename):
        fs, data = wavfile.read(filename)
        spd2 = speedx(data, 4)

        # scaled = np.int16(data/np.max(np.abs(data)) * 32767)
        wavfile.write('test.wav', len(data), data)
        wavfile.write('spd2.wav', len(spd2), spd2)

if __name__ == '__main__':
	# sound = load("r2d2.wav")
    fn = "./media/sad.wav"
    play(fn)
