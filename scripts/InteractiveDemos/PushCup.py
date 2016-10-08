# Edwin Push-Cup Game
# Connor Novak: connor.novak@students.olin.edu
# Project in human-robot interaction: Edwin pushes cup, human pushes cup

# Imports
from __future__ import print_function
import roslib
import cv2
import rospy
import sys
import math
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class cup_pusher:

    # This runs once after the class is instantiated in main
    def __init__(self):

        self.bridge = CvBridge()
        rospy.init_node('push_cup') # This is where you go, "Oh, I remember nodes from the ROS tutorial! Oh wait, that's all I remember". Go back learn it, pleb
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    # Runs once for every reciept of an image from usb_cam
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converts usb cam feed to csv feed; bgr8 is an image encoding
        except CvBridgeError as e:
            print(e)

        self.apply_filter(cv_image)

    # Manages all applications of filters.
    def apply_filter(self, feed):

        # Where all the stuff not in the two functions goes
        blur = cv2.GaussianBlur(feed, (5,5), 0) # Because blurrier is better

        # Cause the functions have to be called somewhere
        step1 = self.detectcup_color(blur)
        step2 = self.detectcup_shape(step1)

        # Feed Display(s) for debug:
        cv2.imshow('Raw Feed (feed)',feed)
        #cv2.imshow('Gaussian Blur Filter (blur)', blur)
        #cv2.imshow('Step 1 Feed: Color (step1)', step1)
        #cv2.imshow('Step 2 Feed: Shape (step2)', step2)

        k = cv2.waitKey(5) & 0xFF

    # Detect Cup using Shape Detection Function
    def detectcup_shape(self, video):

         vidgray = cv2.cvtColor(video,cv2.COLOR_BGR2GRAY) #Changes BGR video to GRAY vidgray

         ret,thresh = cv2.threshold(vidgray,50,90,cv2.THRESH_BINARY_INV)
         contours, h = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

         for cnt in contours:

             approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt, True), True)
             if len(approx)==15 and math.fabs(approx.item(0)-approx.item(2))>10:
                 cv2.drawContours(video,[cnt],0,(0,255,255),-1)

         # Feed Display(s) for debug:
         #cv2.imshow('detectcup_shape: Raw Video(video)',video)
         #cv2.imshow('detectcup_shape: To GRAY Filter (vidgray)',vidgray)
         #cv2.imshow('detectcup_shape: Threshold Filter (thresh)',thresh)

         return video

    # Detect Cup using Color Detection Function
    def detectcup_color(self, video):

         hsv = cv2.cvtColor(video, cv2.COLOR_BGR2HSV) #Changes RGB video to HSV hsv

         # Sets lower and upper bound for filter range (determined via TrackObject.py for red SOLO cup)
         lower_red = np.array([0,132,101])
         upper_red = np.array([183,255,148])

         mask = cv2.inRange(hsv, lower_red, upper_red) # Filters to black all pixels except between range
         res = cv2.bitwise_and(video, video, mask= mask) # Colors filtered section red

         # Feed Display(s) for debug:
         #cv2.imshow('detectcup_color: Raw Video (video)',video)
         cv2.imshow('detectcup_color: To HSV Filter (hsv)', hsv)
         cv2.imshow('detectcup_color: Red Filter (mask)',mask)
         cv2.imshow('detectcup_color: Final Feed (res)',res)
         return res

    # Updating Function
    def run(self):
        r = rospy.Rate(20) # Sets update rate
        while not rospy.is_shutdown():
            r.sleep()

if __name__=='__main__':
    pc = cup_pusher()
    pc.run() # Calls run function
