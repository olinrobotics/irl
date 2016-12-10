#!/usr/bin/env python
import rospy
import cv2
import cv2.cv as cv
import numpy as np
import random
import time
import math
import operator
import itertools

from std_msgs.msg import String
from sensor_msgs.msg import Image
from edwin.msg import Edwin_Shape
from cv_bridge import CvBridge, CvBridgeError

from sight import handwriting_recognition

"""
WritingDemo is a demo in which Edwin does text recognition on a piece of paper with
text written on it. Edwin then writes the text he has recognized on piece of paper placed
on the robot table.
"""
class Game:
    def __init__(self):
        self.draw_pub = rospy.Publisher('draw_cmd', Edwin_Shape, queue_size=10)
        self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
        self.behav_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)

        #handwriting recognizer passed in from brain
        self.recognizer = handwriting_recognition.HandwritingRecognition(True)
        self.recognizer.process_data_svm()
        self.recognizer.train_svm()

        time.sleep(2)
        print "Starting WritingDemo"

    def img_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def run(self):
        print "Starting WritingDemo"
        running = True

        while running:
            word = self.recognizer.get_image_text(self.frame)
            print "WORD IS: ", word

        # cv2.destroyAllWindows()
        print "Finished with WritingDemo :)"

if __name__ == '__main__':
	rospy.init_node('wd_gamemaster', anonymous = True)
	gm = Game()
	gm.run()
