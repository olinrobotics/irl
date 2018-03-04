import tensorflow as tf
import numpy as np

# save file

# remember to define same dtype and same shape when restoring

# W = tf.Variable([[1,2,3], [1,2,3]], dtype=tf.float32, name="weights")
# b = tf.Variable([[4,5,6]], dtype=tf.float32, name="bias")
#
# init = tf.global_variables_initializer()
#
# saver = tf.train.Saver()
#
# with tf.Session() as sess:
#     sess.run(init)
#     save_path = saver.save(sess, "my_net/save_net.ckpt")
#     print "Save to", save_path

############################

# restore variables
# redefine the same dtype and shape of the variables

W = tf.Variable(tf.zeros([2,3]), dtype=tf.float32, name="weights")
b = tf.Variable(tf.zeros([1,3]), dtype=tf.float32, name="bias")

# do not need init step when restoring
saver = tf.train.Saver()
with tf.Session() as sess:
    saver.restore(sess, "my_net/save_net.ckpt")
    print "Weights ", sess.run(W)
    print "bias ", sess.run(b)
