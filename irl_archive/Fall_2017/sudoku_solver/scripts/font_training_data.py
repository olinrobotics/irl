"""
By Khang Vu & Sherrie Shen, 2017
Last modified Dec 13, 2017

This script generates the samples and responses variable for training
The data are font images generated by training_data/font_dataset_generator.py
Related: font_dataset_generator.py
"""
from collections import Counter
import numpy as np
import cv2
import image_helper as imhelp

# Use python list instead of numpy array for faster matrix manipulation
temp_samples = []
temp_responses = []

# Add fonts data from training_data/font_images
for n in range(10):
    for m in range(1, 1000000):
        filename = "training_data/font_images/%d_%d.jpg" % (n, m)

        im_f = cv2.imread(filename, 0)
        if im_f is None:
            break

        im2_f, contours, hierarchy = cv2.findContours(im_f, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) > 50:
                [x, y, w, h] = cv2.boundingRect(cnt)
                if h > 30 and 10 < w < 1.5 * h:

                    # Put the image into a black image
                    black_image = imhelp.put_to_black(im_f, x, y, w, h)

                    # Reshape the image
                    roi = cv2.resize(black_image, (28, 28), interpolation=cv2.INTER_AREA)
                    roi.shape = 784
                    roi = np.float32(roi)

                    # Add to data set
                    for _ in range(30):
                        temp_samples.append(roi)
                        temp_responses.append(n)

# Convert python list to numpy array
samples = np.array(temp_samples, dtype=np.float32)
responses = np.array(temp_responses, dtype=np.int)

print "Count of digits in dataset", Counter(responses)

print "Font data complete"
