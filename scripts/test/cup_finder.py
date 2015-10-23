#!/usr/bin/python
import cv2
import time
import Image
import roslib; roslib.load_manifest('edwin')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from cv2 import cv

def image_callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError, e:
        print e

    img_HSV = cv2.cvtColor(cv_image, cv.CV_BGR2HSV)

    red_Threshed = cv2.inRange(img_HSV, np.array((2,50, 50)), np.array((8,170,200)))
    red_gaussian = cv2.GaussianBlur(red_Threshed, (9,9), 2, 2)

    cv2.imshow("orig", frame)
    cv2.imshow("theshed", red_gaussian)
    c = cv2.waitKey(1)

def publisher():
    rospy.init_node("face_location",anonymous = True)

    pub = rospy.Publisher('cup_forward', String, queue_size = 10)
    rospy.Subscriber('camera/rgb/image_color', Image, image_callback)

    while not rospy.is_shutdown():
        pass

if __name__ == "__main__":
    publisher()
