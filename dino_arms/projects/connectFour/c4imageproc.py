'''
Purpose: to process a board of Connect4
Authors: Hannah Kolano and Kevin Zhang
Contact: hannah.kolano@students.olin.edu
Last edited: 10/7/17
'''

import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt

cap = cv2.VideoCapture(0)

while(cap.isOpened()):
    # Take each frame
    _, frame = cap.read()

    # convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_white = np.array([0, 0, 0])
    upper_white = np.array([255, 50, 255])

    # Threshold the SSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_white, upper_white)

    #Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)
    cv2.imshow('res', res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()

#TODO:
'''get image from Edwin'''

#TODO:
'''find edges of board, normalize the view'''

#TODO:
'''Figure out what the current layout is like'''

#TODO:
'''for debugging, create visual output of board'''

#TODO:
'''Format to send to the machine learning part'''
