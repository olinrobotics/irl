#!/usr/bin/ python

import sys
import pocketsphinx as ps

if __name__ == "__main__":

    hmdir = "/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k"
    lmd   = "/usr/share/pocketsphinx/model/lm/en_US/hub4.5000.DMP"
    dictd = "/usr/share/pocketsphinx/model/lm/en_US/cmu07a.dic"

    psConfig = ps.Decoder.default_config()
    psConfig.set_string('-hmm', hmdir)

    psConfig.set_string('-lm', lmd)
    psConfig.set_string('-dict', dictd)
    speechRec = ps.Decoder(psConfig)

    wavFile = file("o0.wav",'rb')
    speechRec.decode_raw(wavFile)
    result = speechRec.get_hyp()

    print result