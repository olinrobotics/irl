#!/usr/bin/env python

import tensorflow as tf
import numpy as np



learning_rate = .005
seq_len = 15
input_size = 100
timesteps = 15
hidden_size = 128
class_size = 15
max_no = 100
batch_size = 32
n_layers = 2
training_steps = 100000
display_step = 100

x_var = tf.placeholder("float", [None, timesteps, input_size])
y_var = tf.placeholder("float", [None, class_size, max_no])

weights = {
    "out": tf.Variable(tf.random_normal([hidden_size, input_size]))
}
bias = {
    "out": tf.Variable(tf.random_normal([input_size]))
}

def encode(X, seq_len, vocab_size):
    x = np.zeros((len(X),seq_len, vocab_size), dtype=np.float32)
    for ind,batch in enumerate(X):
        for j, elem in enumerate(batch):
            x[ind, j, elem] = 1
    return x

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
    return x, y


def lstm_cell():
    return tf.contrib.rnn.BasicLSTMCell(hidden_size, forget_bias=1.0)

def RNN(x, weights, bias):
    # print x.shape
    # x = tf.unstack(x, timesteps, 1)
    print "x", x.shape
    # x = tf.reshape(x, [-1, timesteps])
    stacked_lstm = tf.contrib.rnn.MultiRNNCell([lstm_cell() for _ in range(n_layers)])

    init_state = stacked_lstm.zero_state(batch_size, dtype=tf.float32)

    outputs, final_state = tf.nn.dynamic_rnn(cell=stacked_lstm, inputs=x, initial_state=init_state, dtype=tf.float32, time_major=False)
    outputs = tf.reshape(outputs, [-1, hidden_size])
    result = tf.matmul(outputs, weights["out"]) + bias["out"]
    result = tf.reshape(result, [-1, timesteps, input_size])

    return result



logits = RNN(x_var, weights, bias)
prediction = tf.nn.softmax(logits)
print "logits", logits.shape

print "shape", y_var.shape
cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=y_var))
optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate)
trainer = optimizer.minimize(cost)

correct_prediction = tf.equal(tf.argmax(prediction, axis=2), tf.argmax(y_var, axis=2))
accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))
pred = tf.argmax(prediction, axis=2)

init = tf.global_variables_initializer()

with tf.Session() as sess:
    sess.run(init)

    for step in range(training_steps):
        trainx, trainy = batch_gen()
        sess.run(trainer, feed_dict={x_var:trainx, y_var:trainy})

        if step%100 == 0:
            loss, acc = sess.run([cost, accuracy], feed_dict={x_var:trainx, y_var:trainy})
            print "Step " + str(step) + ", Minibatch Loss= " + "{:.4f}".format(loss) + ", Training Accuracy= " + "{:.3f}".format(acc)

            testX = np.random.randint(max_no, size=(batch_size, seq_len))
            test = encode(testX, seq_len, max_no) # one-hot encoded version
            # print test.shape
            truth = np.sort(testX[0])
            p = sess.run(pred, feed_dict={x_var:test})
            print "rnn: ", p[0]
            print "np: ", truth
            print " "


    print "Training done"










if __name__=="__main__":

    batch_gen()
