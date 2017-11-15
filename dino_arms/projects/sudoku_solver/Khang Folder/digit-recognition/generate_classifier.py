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
from image_helper import *

"""
This script trains our data and exports the classifier
It takes images from font_data/*.jpg and add them to features and labels
Labels are defined by images' names
"""

# Load the dataset (handwriting data MNIST)
dataset = datasets.fetch_mldata("MNIST Original")

# Extract the features and labels
features = np.array(dataset.data, 'int16')  # 7000 x 784
labels = np.array(dataset.target, 'int')  # 7000 x 1

# Our extra data stored in these variables
digits = []
temp_labels = []


def process_font_image(d):
    # Convert to grayscale and apply Gaussian filtering
    im_gray = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)
    im_gray = cv2.GaussianBlur(im_gray, (5, 5), 0)

    # Threshold the image
    ret, im_th = cv2.threshold(im_gray, 90, 255, cv2.THRESH_BINARY_INV)
    show_image(im_th)
    # Find contours in the image
    im2, contours, hierarchy = cv2.findContours(im_th.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rects = [cv2.boundingRect(ctr) for ctr in contours]

    # This part of the code makes sure to get the right digit region
    if len(rects) > 1:
        max_area = 0
        max_area_index = 0
        index = 0
        for r in rects:
            area = r[2] * r[3]
            if max_area < area:
                max_area = area
                max_area_index = index
            index += 1
        rect = rects[max_area_index]
    else:
        rect = rects[0]

    if rect[3] > rect[2]:
        length = int(rect[3] * 1.6)
    else:
        length = int(rect[2] * 1.6)

    digit_rectangle = im_th[rect[1]:rect[1] + rect[3], rect[0]:rect[0] + rect[2]]
    y_offset = int((length - rect[3]) / 2)
    x_offset = int((length - rect[2]) / 2)
    black_bg = np.full((length, length), 0, dtype='uint8')
    black_bg[y_offset:y_offset + rect[3], x_offset:x_offset + rect[2]] = digit_rectangle

    d = cv2.resize(black_bg, (28, 28), interpolation=cv2.INTER_LINEAR)
    d = np.reshape(d, 784)

    return d.astype('int16')


for i in range(0, 10):
    digit = cv2.imread('font_data/%d.jpg' % i)
    digit = process_font_image(digit)

    # Add [number] times to our training data
    # number = 1000
    # for _ in range(number):
    #     digits.append(digit)
    #     temp_labels.append(i % 10)

# Add our extra data to features and labels
features = np.concatenate((features, digits), 0)
labels = np.concatenate((labels, temp_labels))

# Extract the hog features
list_hog_fd = []
for feature in features:
    fd = hog(feature.reshape((28, 28)), orientations=9, block_norm='L2-Hys', pixels_per_cell=(14, 14),
             cells_per_block=(1, 1), visualise=False)
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
