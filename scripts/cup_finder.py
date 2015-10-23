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

class ImageConverter:
  def __init__(self):
    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)
    self.fgbg = cv2.BackgroundSubtractorMOG()

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    # img_HSV = cv2.cvtColor(cv_image, cv.CV_BGR2HSV)
    crop = cv_image[240:480, 0:640]
    fgmask = self.fgbg.apply(crop)

    lines = cv2.HoughLines(fgmask, 1, np.pi/180, 100, 100, 10)
    if lines != None:
        print lines[0][0][0]
        if 50 < abs(lines[0][0][0]) < 170:
            print "sophie's turn"
        else:
            print "edwin's turn"

    res = cv2.bitwise_and(crop, crop, mask=fgmask)
    cv2.imshow("mog", res)

    cv2.imshow("Image window", fgmask)
    cv2.waitKey(3)


def main():
    ic = ImageConverter()
    rospy.init_node('imageconverter')
    while not rospy.is_shutdown():
        pass

if __name__ == "__main__":
    main()
