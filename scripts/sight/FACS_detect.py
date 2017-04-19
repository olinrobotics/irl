import rospy
import dlib

import dlib_features as dl
import get_features_face as gf

from facial_expression_learning import FACSTrainer
from time import sleep
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class FACSDetect:
    def __init__(self):
        # initialize frame
        self.frame = None

        # get classifiers
        nose_trainer = FACSTrainer(face_region='nose')
        nose_classier = nose_trainer.classifier

        self.classifiers = {'nose': nose_classier}

        # CvBridge to usb_cam, subscribes to usb_cam ros node
        self.bridge = CvBridge()
        rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)

        # ------ATTRIBUTES------
        # boolean to determine if detecting
        self.detect = True
        # new_face boolean to control how many expressions detected
        self.new_face = True
        # detector from dlib_features.py
        self.face_detector = dl.FaceDetect()
        # a window: how neat!
        self.window = dlib.image_window()

        # wait 2 seconds
        sleep(2)

    # converts ros message to numpy
    def img_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            h, w = self.frame.shape[:2]
        except CvBridgeError as e:
            print(e)

    def face_detect(self):
        if self.frame is None:
            return

        # TODO: process here using get_features_face
        # TODO: get landmarks
        # TODO: landmarks = split_landmarks
