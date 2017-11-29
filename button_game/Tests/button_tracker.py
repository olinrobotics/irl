#!/usr/bin/env python

# import the necessary packages
import imutils
import rospy
import Float64
from dino_arms.msg import blue_button, yellow_button, red_button, green_button, all_buttons
import numpy as np
import cv2

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points

buttons = rospy.Publisher('button_data', all_buttons, queue_size=10)

blue_button = blue_button()
red_button = red_button()
green_button = green_button()
yellow_button = yellow_button()

all_buttons = all_buttons()

greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

blueLower = (110, 50, 50)
blueUpper = (130, 255, 255)

yellowLower = (23, 41, 133)
yellowUpper = (40, 150, 255)

redLower = (160, 140, 50)
redUpper = (179, 255, 255)

# grab the reference to the webcam
camera = cv2.VideoCapture(0)

#keep looping
while True:
    #grab the current frame
    (grabbed, frame) = camera.read()

    # resize the frame, blur it, and return
    # it in the HSV color space
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # create a mask for the color green and then
    # perform erodions and dialations to make the
    # tracking more smooth

    mask_blue = cv2.inRange(hsv, blueLower, blueUpper)
    mask_green = cv2.inRange(hsv, greenLower, greenUpper)
    mask_yellow = cv2.inRange(hsv, yellowLower, yellowUpper)
    mask_red = cv2.inRange(hsv, redLower, redUpper)

    masks = [mask_blue, mask_green, mask_yellow, mask_red]

    # mask_total = mask_blue + mask_green + mask_yellow + mask_red"""

    for i in range(4):
        masks[i] = cv2.erode(masks[i], None, iterations=2)
        masks[i] = cv2.dilate(masks[i], None, iterations=2)

        # find contours in the mask and initialize the (x, y) position
        # of the center of the ball
        cnts = cv2.findContours(masks[i].copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # only proceed if at least one contour found
        if len(cnts) > 0:
            # find largest contour area, compute minimum enclosing circle
            # and find its center
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if radius meets minimum size
            if radius > 10:
                if i == 0:
                    blue_center = center
                    blue_radius = radius
                    print(radius)
                elif i == 1:
                    green_center = center
                    green_radius = radius
                    # Publish to green
                elif i == 2:
                    yellow_center = center
                    yellow_radius = radius
                    # Publish to yellow
                elif i == 3:
                    red_center = center
                    red_radius = radius
                    # Publish to red

            # draw circle outline and cente
            cv2.circle(frame, (int(x), int(y)), int(radius), (0,255,255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)



    # show frame to screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF



    # q key will stop loop
    if key == ord("q"):
        break

# turn off camera and close any open windows
camera.release()
cv2.destoryAllWindows
