#!/usr/bin/env python

import rospy
import dlib
import rospkg
import cv2
import numpy as np
import math

import dlib_features as dl
import get_features_face as gf

from facial_expression_learning import FACSTrainer
from time import sleep
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sklearn.externals import joblib


class FACSDetect:
    def __init__(self):
        rospy.init_node('facs', anonymous=True)

        # initialize frame
        self.frame = None

        # define file paths
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        self.params_path = PACKAGE_PATH + '/params/'

        # get classifiers
        nose_classifier = joblib.load(self.params_path + 'nose' + '.pkl')
        mouth_classifier = joblib.load(self.params_path + 'mouth' + '.pkl')
        eyes_classifier = joblib.load(self.params_path + 'eyes' + '.pkl')
        forehead_classifier = joblib.load(self.params_path + 'forehead' + '.pkl')
        cheeks_classifier = joblib.load(self.params_path + 'cheeks' + '.pkl')

        self.classifiers = {'nose': nose_classifier, 'mouth': mouth_classifier,
                            'eyes': eyes_classifier,
                            'forehead': forehead_classifier}

        # CvBridge to usb_cam, subscribes to usb_cam ros node
        self.bridge = CvBridge()
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        print("bridge set up")


        # dlib detector
        self.detector = dlib.get_frontal_face_detector()
        # ------ATTRIBUTES------
        # boolean to determine if detecting
        self.detect = True
        # new_face boolean to control how many expressions detected
        self.new_face = True
        # detector from dlib_features.py
        self.face_detector = dl.FaceDetect()
        # features from get_features_face.py
        self.features = gf.Features()
        # a window: how neat!
        self.window = dlib.image_window()

        # wait 2 seconds
        sleep(2)
        print "ready to FACS"

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
        # convert to gray
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        # get and appy clahe image
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        clahe_image = clahe.apply(gray)

        # crop to biggest face in frame
        faces = self.detector(clahe_image, 1)
        if len(faces) > 0:
            # biggest_area = 0
            # biggest_face = faces[0]
            #
            # for face in faces:
            #     x = face.left()
            #     y = face.top()
            #     w = face.right() - x
            #     h = face.bottom() - y
            #     area = w*h
            #     if area > biggest_area:
            #         biggest_face = gray[y:y+h, x:x+w]
            #         biggest_area = area
            # print('face', biggest_face)
            # get landmarks
            landmarks = self.face_detector.get_landmarks(clahe_image=clahe_image,
                                                         return_type='nparray')
            if len(landmarks) > 0:

                # landmarks = split_landmarks
                split = self.features.split_landmarks(landmarks)
                for region in self.classifiers.keys():
                    unpacked_landmarks = self.features.unpack_landmarks(split[region])

                    # reshapes data and puts in numpy array for classifier
                    data = np.asarray(unpacked_landmarks).reshape(1, -1)

                    # Nose FACS
                    print(self.classifiers[region])
                    probs = self.classifiers[region].predict_proba(X=data)
                    if not math.isnan(probs[0][0]):
                        print(region + ': ')
                        print(probs)
            else:
                print('no landmarks')
        else:
            print("no face")




        # cv2.imshow('Video', self.frame)
        # if face != None:
        #     cv2.imshow('Face', face)



    def run(self):
        while not rospy.is_shutdown():
            if self.detect:
                self.face_detect()


if __name__ == '__main__':
    fd = FACSDetect()
    fd.run()
