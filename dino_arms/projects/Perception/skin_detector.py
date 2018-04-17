"""
USAGE
# python skin_detector.py
# python skin_detector.py --video video/skin_example.mov
"""

import numpy as np

import cv2
import imutils


def has_hand(image):
    # define the upper and lower boundaries of the HSV pixel
    # intensities to be considered 'skin'
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    lower = np.array([0, 48, 80], dtype="uint8")
    upper = np.array([20, 255, 255], dtype="uint8")

    # resize the frame, convert it to the HSV color space,
    # and determine the HSV pixel intensities that fall into
    # the speicifed upper and lower boundaries
    frame = imutils.resize(image, width=400)
    converted = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    skinMask = cv2.inRange(converted, lower, upper)

    # apply a series of erosions and dilations to the mask
    # using an elliptical kernel
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
    skinMask = cv2.erode(skinMask, kernel, iterations=2)
    skinMask = cv2.dilate(skinMask, kernel, iterations=2)

    # blur the mask to help remove noise, then apply the
    # mask to the frame
    skinMask = cv2.GaussianBlur(skinMask, (3, 3), 0)

    count = 0
    for array in skinMask:
        for elm in array:
            if elm == 255:
                count += 1

    if count > 1500:
        print("Hand detected")
        return True
    else:
        return False


if __name__ == '__main__':
    image_path = "test_images/hand6.JPG"
    image = cv2.imread(image_path)

    has_hand(image)
