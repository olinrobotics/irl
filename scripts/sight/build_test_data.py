#!/usr/bin/env python
import rospy
import rospkg
import time

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from scipy import stats
from Character import Character

import cv2
import img_processing as Process

class BuildData:

    def __init__(self):
        cv2.setUseOptimized(True)
        self.img = cv2.imread('test_imgs/digits.png')
        rospy.init_node('handwriting_recognition', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        self.detect = True
        cv2.namedWindow('image')
        cv2.setMouseCallback('image',self.fill_test_data)
        self.test_data = np.zeros((200,200),np.uint8)
        self.test_filled = 0

    def fill_test_data(self, event, x, y, flags, param):
        init_character = 'c'
        path = 'char_data/'
        if event == cv2.EVENT_LBUTTONDOWN:
            for contour in self.numbers:
                if self.test_filled < 100:
                    x_index = (self.test_filled%10)*20
                    y_index = (self.test_filled // 10)*20
                    self.test_data[y_index:y_index+20,x_index:x_index+20] = contour.img
                    self.test_filled += 1
        elif event == cv2.EVENT_RBUTTONDOWN:
            write_path = path + init_character + '.png'
            cv2.imwrite(write_path,self.test_data)
            init_chracter += 1
            self.test_filled = 0
            self.test_data[:,:] = 0

    def img_callback(self, data):
        try:
            self.curr_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


    # Update the current frame
    def update_frame(self):
        self.frame = self.curr_frame

    # display the current image
    def output_image(self):
        new_frame = self.frame
        cv2.imshow('image', new_frame)
        cv2.imshow('out',self.test_data)
        cv2.waitKey(1)

    def run(self):
        r = rospy.Rate(10)
        time.sleep(2)
        while not rospy.is_shutdown():
            self.update_frame()
            self.numbers = Process.get_text_roi(self.frame)
            self.output_image()
            r.sleep()

if __name__ == '__main__':
    hr = BuildData()
    hr.run()
