'''
Purpose: to process a board of Connect4
Authors: Hannah Kolano and Kevin Zhang
Contact: hannah.kolano@students.olin.edu
Last edited: 11/1/17
'''

import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt

'''initialize camera'''
cap = cv2.VideoCapture(0)
_, frame = cap.read()
imgray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) # grayscale image

'''define template'''
template = cv2.imread('templatepic.png')
template = cv2.resize(template, (500,500))
templategray = cv2.cvtColor(template, cv2.COLOR_RGB2GRAY)

img = cv2.medianBlur(imgray,5)
th1 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,20)
__, contours, __ = cv2.findContours(th1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(frame, contours, -1, (0,255,0), 1)

'''
cnt = contours[0]
rect = cv2.minAreaRect(cnt)
box = cv2.boxPoints(rect)
box = np.int0(box)
cv2.drawContours(th1,[box],0,(0,0,255),2)
'''

while(True):
    cv2.imshow('frame', frame)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        destroyAllWindows()
    elif k == ord('s'):
        cv2.imwrite('tempcontour.png', img)

#TODO:
'''find edges of board, normalize the view'''

#TODO:
'''Figure out what the current layout is like'''

#TODO:
'''for debugging, create visual output of board'''

#TODO:
'''Format to send to the machine learning part'''
