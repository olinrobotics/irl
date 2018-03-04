import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt

def add_layer(inputs, in_size, out_size, n_layer,  activation_function=None):
    layer_name = "layer " + str(n_layer)
    with tf.name_scope("layer"):
        with tf.name_scope("weights"):
            Weights = tf.Variable(tf.random_normal([in_size, out_size]), name="W")
            tf.summary.histogram(layer_name+"/weights", Weights)
        with tf.name_scope("biases"):
            bias = tf.Variable(tf.zeros([1,out_size])+ 0.1, name="b")
            tf.summary.histogram(layer_name+"/biases", bias)
        with tf.name_scope("Wx_b"):
            Wx_plus_b = tf.matmul(inputs, Weights) + bias
        if activation_function is None:
            outputs = Wx_plus_b
        else:
            outputs = activation_function(Wx_plus_b)
        tf.summary.histogram(layer_name+"/outputs", outputs)

        return outputs


# Make up some real data
x_data = np.linspace(-1, 1, 1000)[:,np.newaxis]
noise = np.random.normal(0, 0.05, x_data.shape)
y_data = np.square(x_data) - 0.5 + noise

# plt.scatter(x_data, y_data)
# plt.show()

# define placeholders for inputs to network
with tf.name_scope("inputs"):
    xs = tf.placeholder(tf.float32, [None, 1], name="x_input")
    ys = tf.placeholder(tf.float32, [None, 1], name="y_input")

# add hidden layer
l1 = add_layer(xs, 1, 10, 0, activation_function=tf.nn.relu)
# add output layer
prediction = add_layer(l1, 10, 1, 1, activation_function=None)

# the error between prediction and real data
with tf.name_scope("loss"):
    loss = tf.reduce_mean(tf.reduce_sum(tf.square(ys-prediction), reduction_indices=[1]), name="loss")
    tf.summary.scalar("loss", loss)
with tf.name_scope("train"):
    train_step = tf.train.AdamOptimizer(0.1).minimize(loss)

# important step
init = tf.global_variables_initializer()

fig = plt.figure()
ax = fig.add_subplot(1,1,1)
ax.scatter(x_data, y_data)
plt.ion()
plt.show()

with tf.Session() as sess:
    merge = tf.summary.merge_all()
    writer = tf.summary.FileWriter("logs/", sess.graph)
    sess.run(init)
    for i in range(1000):
        # training
        sess.run(train_step, feed_dict={xs:x_data, ys:y_data})
        if i%50 == 0:
            try:
                ax.lines.remove(lines[0])
            except Exception:
                pass
            # print sess.run(loss,  feed_dict={xs:x_data, ys:y_data})
            prediction_value = sess.run(prediction, feed_dict={xs:x_data})
            lines = ax.plot(x_data, prediction_value, 'r-', lw=5)
            result = sess.run(merge, feed_dict={xs:x_data, ys:y_data})
            writer.add_summary(result, i)

            plt.pause(0.1)
