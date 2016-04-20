#!/usr/bin/env python
import rospy
import rospkg

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import time
import math
import numpy as np
import os, sys

import cv2
import cv2.cv as cv

class FaceTracker:
    def __init__(self):
        rospy.init_node('face_tracking', anonymous=True)
        self.pub = rospy.Publisher('/arm_cmd', String, queue_size=10)
        rospy.Subscriber("all_control", String, self.cmd_callback)

        self.bridge = CvBridge()
        rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)

        self.detect = True
        self.face_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml')

        self.ideal_x = 0
        self.ideal_y = 0

    def img_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def cmd_callback(self, data):
        print "GOT: ", data.data
        if "ft stop" in data.data:
            self.detect = False
        elif "ft go" in data.data:
            self.detect = True

    def face_tracking(self):
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        largest_face = (0, 0, 0, 0) #x, y, w, h
        for (x,y,w,h) in faces:
            cv2.rectangle(self.frame,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = self.frame[y:y+h, x:x+w]
            roi_color = self.frame[y:y+h, x:x+w]

            if w*h > largest_face[2]*largest_face[3]:
                largest_face = (x, y, w, h)



        cv2.imshow("fr", self.frame)
        cv2.waitKey(1)

    def run(self):
        r = rospy.Rate(10)
        time.sleep(2)
        while not rospy.is_shutdown():
            if self.detect:
                self.face_tracking()
            r.sleep()

if __name__ == '__main__':
    ft = FaceTracker()
    ft.run()
