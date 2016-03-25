import sys,os
import pyaudio
import wave
import aiml
#
#
# MODELDIR = "pocketsphinx/model"
# DATADIR = "pocketsphinx/test/data"
#
# # Create a decoder with certain model
# config = Decoder.default_config()
# config.set_string('-hmm', path.join(MODELDIR, 'en-us/en-us'))
# config.set_string('-lm', path.join(MODELDIR, 'en-us/en-us.lm.bin'))
# config.set_string('-dict', path.join(MODELDIR, 'en-us/cmudict-en-us.dict'))

hmdir = "../../tools/pocketsphinx/model/en-us/en-us"
lmd   = "../../tools/pocketsphinx/model/en-us/en-us.lm.bin"
dictd = "../../tools/pocketsphinx/model/en-us/cmudict-en-us.dict"


def decodeSpeech(hmmd,lmdir,dictp,wavfile):

    import pocketsphinx as ps
    import sphinxbase
    from os import environ, path

    MODELDIR = "../../tools/pocketsphinx/model"
    DATADIR = "../../tools/pocketsphinx/test/data"

    # Create a decoder with certain model
    config = ps.Decoder.default_config()
    config.set_string('-hmm', path.join(MODELDIR, 'en-us/en-us'))
    config.set_string('-lm', path.join(MODELDIR, 'en-us/en-us.lm.bin'))
    config.set_string('-dict', path.join(MODELDIR, 'en-us/cmudict-en-us.dict'))
    speechRec = ps.Decoder(config)

    # speechRec = ps.Decoder(hmm = hmmd, lm = lmdir, dict = dictp)
    wavFile = file(wavfile,'rb')
    wavFile.seek(44)
    speechRec.decode_raw(wavFile)
    result = speechRec.get_hyp()

    return result[0]

recognised = decodeSpeech(hmdir,lmd,dictd,"o0.wav")
print recognised


# CHUNK = 1024
# FORMAT = pyaudio.paInt16
# CHANNELS = 1
# RATE = 16000
# RECORD_SECONDS = 4
# #
# # k = aiml.Kernel()
# # k.learn("wildcard.aiml")
#
# for x in range(10):
#     fn = "o"+str(x)+".wav"
#     p = pyaudio.PyAudio()
#     stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
#     print("* recording")
#     frames = []
#     for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
#         data = stream.read(CHUNK)
#         frames.append(data)
#     print("* done recording")
#     stream.stop_stream()
#     stream.close()
#     p.terminate()
#     wf = wave.open(fn, 'wb')
#     wf.setnchannels(CHANNELS)
#     wf.setsampwidth(p.get_sample_size(FORMAT))
#     wf.setframerate(RATE)
#     wf.writeframes(b''.join(frames))
#     wf.close()
#     wavfile = fn
#     # recognised = decodeSpeech(hmdir,lmd,dictd,wavfile)
#     # reply = k.respond(recognised)
#     # cm = 'espeak -s 155 "'+reply+'"'
#     # os.system(cm)
