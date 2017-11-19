#!/usr/bin/python

import numpy as np
import image_helper as imhelper
import cv2
from skimage.feature import hog
from sklearn.externals import joblib
from image_helper import *


def recognize(classifer_path=None, digit_image=None, is_shown=True):
    # Load the classifier
    clf, pp = joblib.load(classifer_path)

    # Read the input image
    im = cv2.imread(digit_image)

    # Convert to grayscale and apply Gaussian filtering
    im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    im_gray = cv2.GaussianBlur(im_gray, (5, 5), 0)

    # Threshold the image
    ret, im_th = cv2.threshold(im_gray, 90, 255, cv2.THRESH_BINARY_INV)
    show_image(im_th)
    # Find contours in the image
    im2, contours, hierarchy = cv2.findContours(im_th.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rects = [cv2.boundingRect(ctr) for ctr in contours]

    # For each rectangular region, calculate HOG features and predict
    # the digit using Linear SVM.
    for rect in rects:

        # Draw the rectangles
        cv2.rectangle(im, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 3)

        # Make the rectangular region around the digit
        if rect[3] > rect[2]:
            length = int(rect[3] * 1.6)
        else:
            length = int(rect[2] * 1.6)

        digit_rectangle = im_th[rect[1]:rect[1] + rect[3], rect[0]:rect[0] + rect[2]]
        y_offset = int((length - rect[3]) / 2)
        x_offset = int((length - rect[2]) / 2)
        black_bg = np.full((length, length), 0, dtype='uint8')
        black_bg[y_offset:y_offset + rect[3], x_offset:x_offset + rect[2]] = digit_rectangle

        # Resize the image
        roi = cv2.resize(black_bg, (28, 28), interpolation=cv2.INTER_AREA)
        roi = cv2.dilate(roi, (3, 3))

        # Calculate the HOG features
        roi_hog_fd = hog(roi, orientations=9, pixels_per_cell=(14, 14), block_norm='L2-Hys',
                         cells_per_block=(1, 1), visualise=False)
        roi_hog_fd = pp.transform(np.array([roi_hog_fd], 'float64'))

        # Predict image
        nbr = clf.predict(roi_hog_fd)

        # Put yellow text (predicted digit) above  digit
        cv2.putText(im, str(int(nbr[0])), (rect[0], rect[1]), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 255, 255), 3)

    if is_shown:
        imhelper.show_image(im)


# recognize("digits_cls_8000.pkl", "test_images/photo_4.jpg")
recognize("digits_cls.pkl", "test_images/photo_1.jpg")
