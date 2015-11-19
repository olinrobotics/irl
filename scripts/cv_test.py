#!/usr/bin/env python
import roslib
roslib.load_manifest('edwin')
from std_msgs.msg import String
import rospy
import cv2
import cv2.cv as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter:
  def __init__(self):
    # self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    self.cv_image = None

  def callback(self,data):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    # cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError, e:
    #   print e


def main():
	ic = ImageConverter()
	rospy.init_node("cv_tester", anonymous=True)
	while not rospy.is_shutdown():
		if ic.cv_image == None:
			continue
		frame = ic.cv_image
		x_cen = []
		y_cen = []
		hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		blue_image = cv2.inRange(hsv_image, (0,50,50), (8,170,200))

		circles = cv2.HoughCircles(blue_image, cv.CV_HOUGH_GRADIENT,1,20,
		param1=50,param2=30,minRadius=20,maxRadius=100)

		if circles != None:
			circles = np.uint16(np.around(circles))
			for i in circles[0,:]:
				# draw the outer circle
				x_cen.append(i[0])
				y_cen.append(i[1])
				cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
				# draw the center of the circle
				cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)

		if len(x_cen) != 0:
			x_avg = sum(x_cen)/len(x_cen)
			y_avg = sum(y_cen)/len(y_cen)

			cv2.circle(frame, (x_avg, y_avg), 5, (0,0,255), 3)
			print (x_avg, y_avg)

		cv2.imshow("camera", frame)
		cv2.imshow("blue image", blue_image)
		c = cv2.waitKey(1)

if __name__ == '__main__':
	main()\