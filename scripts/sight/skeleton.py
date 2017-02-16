#!/usr/bin/env python
import rospy
import math
import numpy as np
from std_msgs.msg import String, Int16
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from edwin.msg import *
import time
import tf
import cv2
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError

class Skeleton(object):
    """
    Class to hold and use skeleton of users
    """
    def __init__(self):
        rospy.init_node("skeleton", anonymous=True)
        rospy.Subscriber("/skeleton_markers", Marker, self.constructSkeleton, queue_size=10)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.renderImage, queue_size=10)

        self.listener = tf.TransformListener()


        self.bridge = CvBridge()
        self.body_points = None
        self.head = None
        self.running = True


    def constructSkeleton(self, skeleton):
        """
        Parses out skeleton_markers data

        Body points is a list of 15 markers that denotes body parts, as per the skeleton
        They are in this order in the list:

        0 - head
        1 - neck
        2 - torso

        3 - right shoulder
        4 - right elbow
        5 - right hand

        6 - left shoulder
        7 - left elbow
        8 - left hand

        9 - right hip
        10 - right knee
        11 - right foot

        12 - left hip
        13 - left knee
        14 - left foot
        """
        self.body_points = skeleton.points
        self.head = self.body_points[0]
        print "raw", self.head
        self.head = self.transform_skel2kinect()
        print "processed", self.head
        print " "

    def renderImage(self, image):
        """
        Renders an image using opencv
        """
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        imagex = int(self.head[0]) + 320
        imagey = 240 - int(self.head[1])
        cv2.circle(cv_image, (imagex, imagey), 10, 255, thickness = -1)


        cv2.imshow("chicken", cv_image)
        cv2.waitKey(3)


    def transform_skel2kinect(self):
        """
        makes a transformation from the coordinates of the skeleton to the coordinates
        of the Kinect image
        """
        try:
            (trans,rot) = self.listener.lookupTransform("/camera_depth_frame","/head_1" , rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            trans = None
            print e

        return trans



    def run(self):
        """
        main run function for edwin
        """
        print "running presence skeleton"
        r = rospy.Rate(10)
        time.sleep(1)

        cv2.namedWindow("chicken")

        while self.running:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            r.sleep()

if __name__ == "__main__":
    skull = Skeleton()
    skull.run()
