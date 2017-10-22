#!/usr/bin/python

# Import the modules
import cv2
from sklearn.externals import joblib
from sklearn import datasets
from skimage.feature import hog
from sklearn.svm import LinearSVC
from sklearn import preprocessing
import numpy as np
from collections import Counter

# Load the dataset/ handwriting data MNIST
dataset = datasets.fetch_mldata("MNIST Original")

# Extract the features and labels
features = np.array(dataset.data, 'int16')  # 7000 x 784
labels = np.array(dataset.target, 'int')  # 7000 x 1

# TODO: Generate and add more data to features and labels
# Our extra data stored in these variables
digits = []
temp_labels = []

for i in range(0, 10):
    digit = cv2.imread('font_data/%d.jpg' % i, 0)
    digit = cv2.resize(digit, (28, 28), interpolation=cv2.INTER_LINEAR)
    digit = cv2.bitwise_not(digit)
    digit = np.reshape(digit, 784)
    # Add number_data to our training data
    number_data = 10000
    for _ in range(number_data):
        digits.append(digit)
        temp_labels.append(i)

# Add our extra data to features and labels
features = np.concatenate((features, digits), 0)
labels = np.concatenate((labels, temp_labels))

# Extract the hog features
list_hog_fd = []
for feature in features:
    fd = hog(feature.reshape((28, 28)), orientations=9, pixels_per_cell=(14, 14), cells_per_block=(1, 1),
             visualise=False)
    list_hog_fd.append(fd)

hog_features = np.array(list_hog_fd, 'float64')

# Normalize the features
pp = preprocessing.StandardScaler().fit(hog_features)
hog_features = pp.transform(hog_features)

print "Count of digits in dataset", Counter(labels)

# Create an linear SVM object
clf = LinearSVC()

# Perform the training
clf.fit(hog_features, labels)

# Save the classifier
joblib.dump((clf, pp), "digits_cls.pkl", compress=3)
