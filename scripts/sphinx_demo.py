#!/usr/bin/env python2
from pocketsphinx import *

hmm = "/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k"
lm   = "/usr/share/pocketsphinx/model/lm/en_US/hub4.5000.DMP"
dic = "/usr/share/pocketsphinx/model/lm/en_US/cmu07a.dic"


config = Decoder.default_config()
config.set_string('-hmm', hmm)
config.set_string('-lm', lm)
config.set_string('-dict', dic)
#config.set_string('-logfn', '/dev/null')

print "setup decoder"
decoder = Decoder(config)
print "setup down"

p = pyaudio.PyAudio()

stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
stream.start_stream()
in_speech_bf = True
decoder.start_utt('')
while True:
    buf = stream.read(1024)
    if buf:
        print "process decoder"
        decoder.process_raw(buf, False, False)
        print "process done"
        try:
            if  decoder.hyp().hypstr != '':
                print 'Partial decoding result:', decoder.hyp().hypstr
        except AttributeError:
            pass
        if decoder.get_in_speech():
            sys.stdout.write('.')
            sys.stdout.flush()
        if decoder.get_in_speech() != in_speech_bf:
            in_speech_bf = decoder.get_in_speech()
            if not in_speech_bf:
                decoder.end_utt()
                try:
                    if  decoder.hyp().hypstr != '':
                        print 'Stream decoding result:', decoder.hyp().hypstr
                except AttributeError:
                    pass
                decoder.start_utt('')
    else:
        break
decoder.end_utt()
print 'An Error occured:', decoder.hyp().hypstr