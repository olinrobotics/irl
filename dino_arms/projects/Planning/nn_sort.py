#!/usr/bin/env python

import tensorflow as tf
import numpy as np



learning_rate = .005
seq_len = 15
input_size = 1
timesteps = 15
hidden_size = 128
class_size = 15
max_no = 100
batch_size = 32
n_layers = 2
training_steps = 10000
display_step = 100

x_var = tf.placeholder("float", [None, timesteps, input_size])
y_var = tf.placeholder("float", [None, class_size])

weights = {
    "out": tf.Variable(tf.random_normal([hidden_size, class_size]))
}
bias = {
    "out": tf.Variable(tf.random_normal([class_size]))
}


def batch_gen():
    x = np.zeros((batch_size, seq_len, max_no), dtype=np.float32)
    y = np.zeros((batch_size, seq_len, max_no), dtype=np.float32)

    X = np.random.randint(max_no, size=(batch_size, seq_len))
    Y = np.sort(X, axis=1)

    for ind, batch in enumerate(X):
        for j, elem in enumerate(batch):
            x[ind, j, elem] = 1

    for ind, batch in enumerate(Y):
        for j, elem in enumerate(batch):
            y[ind, j, elem] = 1

    print x
    print y


def create_model(x, weights, bias):

    x = tf.unstack(x, timesteps, 1)
    lstm_cell = tf.contrib.rnn.BasicLSTMCell(hidden_size, forget_bias=1.0)
    outputs, states = tf.contrib.rnn.static_rnn(lstm_cell, x, dtype=tf.float32)

    return tf.matmul(outputs[-1], weights["out"]) + bias["out"]

logits = create_model(x_var, weights, bias)
prediction = tf.nn.softmax(logits)

loss_op = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=y_var))
optimizer = tf.train.GradientDescentOptimizer(learning_rate=learning_rate)
trainer = optimizer.minimize(loss_op)

correct_prediction = tf.equal(tf.argmax(prediction, 1), tf.argmax(Y, 1))
accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))

init = tf.global_variables_initializer()

with tf.Sessions as sess:
    






if __name__=="__main__":

    batch_gen()
