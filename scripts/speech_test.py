import sys,os
import pyaudio
import wave

import pocketsphinx as ps
import sphinxbase


hmdir = "/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k"
lmd   = "/usr/share/pocketsphinx/model/lm/en_US/hub4.5000.DMP"
dictd = "/usr/share/pocketsphinx/model/lm/en_US/cmu07a.dic"

def decodeSpeech(hmmd,lmdir,dictp,wavfile):
    psConfig = ps.Decoder.default_config()
    psConfig.set_string('-hmm', hmdir)

    psConfig.set_string('-lm', lmd)
    psConfig.set_string('-dict', dictd)
    speechRec = ps.Decoder(psConfig)

    # speechRec = ps.Decoder(hmm = hmmd, lm = lmdir, dict = dictp)
    wavFile = file(wavfile,'rb')
    wavFile.seek(44)
    speechRec.decode_raw(wavFile)
    result = speechRec.hyp()

    return (result.hypstr, '')

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
RECORD_SECONDS =5

for x in range(10):
    fn = "o"+str(x)+".wav"
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
    print("* recording")
    frames = []
    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)
    print("* done recording")
    stream.stop_stream()
    stream.close()
    p.terminate()
    wf = wave.open(fn, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()
    wavfile = fn
    recognised = decodeSpeech(hmdir,lmd,dictd,wavfile)
    print recognised
    cm = 'espeak "'+recognised+'"'
    os.system(cm)