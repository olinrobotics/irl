#!/usr/bin/env python
import rospy
import cv2
import cv2.cv as cv
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from edwin.msg import Edwin_Shape
from cv_bridge import CvBridge, CvBridgeError

class visualmenu:
    #The goal is to have someone put their hand under the camera, and then
    #the camera will read which portion of the screen the hand is in.
    #If the person keeps their hand relatively still for 3 seconds,
    #determined by increasing the contour area by 50%, then the
    #
    def _init_(self):
        self.choice_pub = rospy.Publisher("choice_cmd", String, queue_size=10)

        self.game_state = rospy.Subscriber("all_control", String, self.cmd_callback)


        self.bridge = CvBridge()
        rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)
        self.image_sub = rospy.subscriber("usb_cam/camera_raw", Image, self.image_callback)

        self.range1 = [[-20,20], [-20,20]] #minx, maxx, miny, maxy
        self.range2 = [[20,40], [20,40]]

    def img_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            h, w = self.frame.shape[:2]
        except CvBridgeError as e:
            print(e)

    def cmd_callback(self, data):
        print "GOT: ", data.data
        if "vm stop" in data.data:
            self.detect = False
        elif "vm go" in data.data:
            self.started_tracking = time.time()
            self.detect = True

    def findHand(self):
        #Image pre-processing
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(5,5),0)
        ret,thresh1 = cv2.threshold(blur,70,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        #Find the hand
        contours, hierarchy = cv2.findContours(thresh1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for i in range(len(contours)):
            cont=contours[i]
            area = cv2.contourArea(cnt) #Theory that the largest contour is a hand.
            if(area>max_area):
                max_area=area
                ci=i
        cont=contours[ci]

        #find center coordinate of the hand, or rather, the centroids
        m = cv2.moments(cont)
        cx = int(m['m10']/m['m00'])
        cy = int(m['m01']/m['m00'])

        coords = [cx, cy, cont]
        return coords

    def check_pos(self):
