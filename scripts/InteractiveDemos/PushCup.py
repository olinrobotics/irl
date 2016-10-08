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
        self.cup_x_prev = 640/2
        self.cup_y_prev = 480/2
    # Runs once for every reciept of an image from usb_cam
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converts usb cam feed to csv feed; bgr8 is an image encoding
        except CvBridgeError as e:
            print(e)
        self.apply_filter(cv_image)

    # Manages all applications of filters.
    def apply_filter(self, feed):

        # Where all the stuff not in the detectcup function goes
        blur = cv2.GaussianBlur(feed, (5,5), 0) # Because blurrier is better

        # Cause the functions have to be called somewhere
        contour, contours = self.contour_cup(blur)
        self.calculate(contour, contours)
        # Feed Display(s) for debug:
        #cv2.imshow('Raw Feed (feed)',feed)
        #cv2.imshow('Gaussian Blur Filter (blur)', blur)
        #cv2.imshow('Contour Filter (contour)', contour)

        k = cv2.waitKey(5) & 0xFF

    # Cup Detection & Contouring Function
    def contour_cup(self, video):
         contour = video # Duplicate video feed so as to display both raw footage and final contoured footage
         vidgray = cv2.cvtColor(video,cv2.COLOR_BGR2GRAY) #Changes BGR video to GRAY vidgray
         ret,thresh = cv2.threshold(vidgray,100,200,cv2.THRESH_BINARY_INV)
         contours, h = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
         cv2.drawContours(contour, contours, -1, (0,255,0), 3)

         # Feed Display(s) for debug:
         #cv2.imshow('contour_cup: Raw Video(video)',video)
         #cv2.imshow('detectcup_shape: To GRAY Filter (vidgray)',vidgray)
         #cv2.imshow('detectcup_shape: Threshold Filter (thresh)',thresh)
         #cv3.imshow('contour_cup: Final Video(contour)',contour)

         return contour, contours

    # Center & Movement Detection Function
    def calculate(self, contour, contours):
        video = contour
        cnt = contours[0]
        moments = cv2.moments(cnt)
        if moments['m00']!=0:

            # Calculates xy values of centroid and draws circle on it
            self.cup_x = int(moments['m10']/moments['m00'])
            self.cup_y = int(moments['m01']/moments['m00'])
            cv2.circle(video,(self.cup_x,self.cup_y),5,(255, 0, 0),-1)

            # Check if cup has moved significantly
            if abs(self.cup_x - self.cup_x_prev) >= 10:
                self.cup_moved = True
            else:
                self.cup_moved = False

            print ("x = ",self.cup_x,"| y = ",self.cup_y,"| cup_moved = ",self.cup_moved)
            self.cup_x_prev = self.cup_x
            self.cup_y_prev = self.cup_y

        # Feed Display(s) for debug:
        #cv2.imshow('calculate: Raw Video(contour)',contour)
        cv2.imshow('calculate: Centroid Draw(video)',video)

    def run(self):
        r = rospy.Rate(20) # Sets update rate
        while not rospy.is_shutdown():
            r.sleep()

if __name__=='__main__':
    pc = cup_pusher()
    pc.run() # Calls run function
