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

        self.bridge = CvBridge()
        self.body_points = None
        self.head = None
        self.running = True


    def constructSkeleton(self, skeleton):
        """
        Parses out skeleton_markers data

        Body points is a list of 15 markers that denotes body parts, as per the skeleton
        They are in this order in the list:

        1 - head
        2 - neck
        3 - torso

        4 - left shoulder
        5 - left elbow
        6 - left hand

        7 - right shoulder
        8 - right elbow
        9 - right hand

        10 - left hip
        11 - left knee
        12 - left foot

        13 - right hip
        14 - right knee
        15 - right foot
        """
        self.body_points = skeleton.points
        self.head = self.body_points[0]
        print self.head


    def renderImage(self, image):
        """
        Renders an image using opencv
        """
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        cv2.circle(cv_image, (50,50), 10, 255, thickness = -1)
        cv2.circle(cv_image, (640, 480), 10, 255, thickness = -1)
        cv2.circle(cv_image, (320, 240), 10, 255, thickness = -1)

        cv2.imshow("chicken", cv_image)
        cv2.waitKey(3)


    def transform_skel2image(self, x, y, z):
        """
        makes a transformation from the coordinates of the skeleton to the coordinates
        of the Kinect image
        """
        #TODO


        pass

        """
        ask sophie if a static transform will work, and if so, using what units
        """




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
