'''
A sequence to sequence learning implementation for sorting a list of numbers.
-----------------------------------------------------
usage: sort_seq.py [-h] [-c] [-v] epochs
positional arguments:
  epochs            set number of training epochs
optional arguments:
  -h, --help        show this help message and exit
  -c, --continuing  continue training the saved model
  -v, --verbose     verbose mode
  -----------------------------------------------------
'''

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('-c', '--continuing', action='store_true',
                    help='continue training the saved model')
parser.add_argument('-v', '--verbose', action='store_true',
                    help='verbose mode')
parser.add_argument('epochs', type=int,
                    help='set number of training epochs')
args = parser.parse_args()
if args.verbose:
    verbose = True
else:
    verbose = False
epochs = args.epochs

import os
import numpy as np

# suppress the TensorFlow warning
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'

from keras.models import Sequential
from keras.layers.core import Activation, RepeatVector, Dense, Dropout
from keras.layers.wrappers import TimeDistributed
from keras.layers import LSTM
from keras.models import load_model


class colors:
    ok = '\033[92m'
    fail = '\033[91m'
    close = '\033[0m'


# encode a given integer sequence into RNN compatible format (one-hot representation)
def encode(X,seq_len, vocab_size):
    x = np.zeros((len(X),seq_len, vocab_size), dtype=np.float32)
    for ind,batch in enumerate(X):
        for j, elem in enumerate(batch):
            x[ind, j, elem] = 1
    return x


# generate a stream of inputs for training
def batch_gen(batch_size=32, seq_len=10, max_no=100):
    # Randomly generate a batch of integer sequences (X) and its sorted counterpart (Y)
    x = np.zeros((batch_size, seq_len, max_no), dtype=np.float32)
    y = np.zeros((batch_size, seq_len, max_no), dtype=np.float32)

    while True:
	# Generates a batch of input
        X = np.random.randint(max_no, size=(batch_size, seq_len))
        Y = np.sort(X, axis=1)

        for ind,batch in enumerate(X):
            for j, elem in enumerate(batch):
                x[ind, j, elem] = 1

        for ind,batch in enumerate(Y):
            for j, elem in enumerate(batch):
                y[ind, j, elem] = 1

        yield x, y
        x.fill(0.0)
        y.fill(0.0)


def create_model(seq_len, max_no, n_layers, hidden_size):
    model = Sequential()
    # the encoder LSTM
    model.add(LSTM(hidden_size, input_shape=(seq_len, max_no)))
    # in next layer, repeat the input seq_len times
    model.add(RepeatVector(seq_len))
    # decoder RNN, which will return output sequence
    for _ in range(n_layers):
        model.add(LSTM(hidden_size, return_sequences=True))
    model.add(Dropout(0.5))
    model.add(TimeDistributed(Dense(max_no)))
    model.add(Activation('softmax'))
    return model


if __name__ == "__main__":

    batch_size=32
    seq_len = 15 # number of elements in sequence to sort
    max_no = 100 # upper range of the numbers in sequence
    hidden_size = 128
    n_layers = 2

    # To continue training from a previous model
    if args.continuing:
        print('Continuing training from loaded model...')
        model = load_model('model.h5')
    else:
        model = create_model(seq_len, max_no, n_layers, hidden_size)

        model.compile(loss='categorical_crossentropy', optimizer='adam',
                      metrics=['accuracy'])

    for ind,(X,Y) in enumerate(batch_gen(batch_size, seq_len, max_no)):
        loss, acc = model.train_on_batch(X, Y)
        # check the model performance after each 100th iteration
        if ind % 100 == 0:
            testX = np.random.randint(max_no, size=(1, seq_len))
            test = encode(testX, seq_len, max_no) # one-hot encoded version
            loss_str = '{:4.3f}'.format(loss)
            acc_str = '{:4.3f}'.format(acc)
            print('Epoch: {ind:{8}} of {epochs}  -- loss: {loss_str}, accuracy: {acc_str}')

            if verbose:
                y = model.predict(test, batch_size=1)
                np_sorted = np.sort(testX)[0]
                rnn_sorted = np.argmax(y, axis=2)[0]
                is_equal = np.array_equal(np_sorted, rnn_sorted)
                if is_equal:
                    print(colors.ok+'-----CORRECTLY SORTED-----'+colors.close)
                else:
                    print(colors.fail+'-----incorrectly sorted-----'+colors.close)

                print(np_sorted, ': sorted by NumPy algorithm')
                print(rnn_sorted, ': sorted by trained RNN')
                print("\n")

        if ind > epochs:
            # Save trained model
            model.save('model.h5')
            break
