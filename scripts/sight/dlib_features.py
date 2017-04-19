#!/usr/bin/env python

import rospkg
import rospy

import sys
import os
import cv2
import dlib
import glob
from skimage import io
import numpy as np

from time import sleep
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class FaceDetect:

    def __init__(self):
        # definines file paths
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        self.predictor_path = PACKAGE_PATH + \
            '/params/shape_predictor_68_face_landmarks.dat'
        self.faces_folder_path = PACKAGE_PATH + '/params'

        # def of attributes
        self.detect = True
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor(self.predictor_path)
        self.debug = False

    def get_landmarks(self, clahe_image, return_type='list'):
        # code from:
        # http://www.paulvangent.com/2016/08/05/emotion-recognition-using-facial-landmarks/
        # detect faces in the image
        detections = self.detector(clahe_image, 1)

        # for each detected face
        for k, d in enumerate(detections):

            # Get coordinates
            shape = self.predictor(clahe_image, d)

            xlist = []
            ylist = []
            landmarks = []

            # For each point, draw a red circle with thickness2 on
            # the original frame
            for i in range(1, 68):  # There are 68 landmark points on each face
                if self.debug is True:
                        cv2.circle(self.frame, (shape.part(i).x,
                                                shape.part(i).y), 1,
                                   (0, 0, 255), thickness=2)
                        # cv2.putText(img, text, org, fontFace, fontScale,
                        # color[,thickness[, lineType[, bottomLeftOrigin]]])
                        org = (shape.part(i).x, shape.part(i).y)
                        fontFace = cv2.FONT_HERSHEY_PLAIN
                        cv2.putText(self.frame, str(i), org, fontFace, 1,
                                    (255, 225, 225), 1, 8)
                xlist.append(float(shape.part(i).x))
                ylist.append(float(shape.part(i).y))

            # Store all landmarks in one list in the format x1,y1,x2,y2,etc.
            if return_type == 'list':
                for x, y in zip(xlist, ylist):
                    landmarks.append(x)
                    landmarks.append(y)
            if return_type == 'nparray':
                for i in range(len(xlist)-1):
                    landmarks.append(xlist[i], ylist[i])
        if len(detections) > 0:
            return landmarks
        # If no faces are detected, return error message to other function to
        # handle
        else:
            return 'error'

    # def smile_detect(self, landmarks):

    # landmarks in form x1,y1,x2,y2 -> index = (X) landmark_num*2-2,
    # (Y) landmark_num*2-1
    # LANDMARK TO INDEX FUNCTIONS
    def landmarkX(self, landmark):
        return landmark*2 - 2

    def landmarkY(self, landmark):
        return landmark*2 - 1

    def smile_detect(self, landmarks):
        # SMILE DETECTION
        # mouth_landmarks: 48-68, corners: 48, 54, top of mouth: 50, 51,
        # bottom of mouth: 57, 58

        # landmark points in format [x1,y1]
        left_corner_mouth_y = landmarks[95]
        right_corner_mouth_y = landmarks[107]
        bottom_mouth_y = landmarks[113]
        top_mouth_y = landmarks[99]

        mouth_height = top_mouth_y - bottom_mouth_y
        threshold = mouth_height/2
        lip_line = (left_corner_mouth_y + right_corner_mouth_y)/2
        lip_corner_height = top_mouth_y - lip_line

        #smile_msg = 'False'

        if lip_corner_height > threshold:
            #smile_msg = 'True'
            return True

        return False

    def face_detect(self):
        if self.frame is None:
            return

        # converts feed to grayscale
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        clahe_image = clahe.apply(gray)

        #gets face landmarks
        #landmarks in form x1,y1,x2,y2 -> index = (X) landmark_num*2-2, (Y) landmark_num*2-1
        landmarks = self.get_landmarks(clahe_image)
        if landmarks == "error":
            return

        # #Display the frame
        cv2.imshow("image", self.frame)

        #CENTER OF FACE
        center_face_y = landmarks[self.landmarkY(28)]
        center_face_x = landmarks[self.landmarkX(28)]

        #DIMENSIONS OF FACE
        #bottom center of face: 8, right center of face: 16
        height_face = (center_face_y - landmarks[self.landmarkY(8)])*2
        width_face = (landmarks[self.landmarkX(16)] - center_face_x)
        area_face = height_face * width_face

        smile_msg = "False"

        if self.smile_detect(landmarks):
            smile_msg = "True"

        # message for Publisher
        msg = ''
        # smile_msg = 'False'

        print smile_msg
        print "------"
        #
        # #BIGGEST FACE
        # biggestArea = 0;
        # if area_face > biggestArea:
        #     area_face = biggestArea
        msg = str(center_face_x) + ',' + str(center_face_y) + ':' + smile_msg

        if cv2.waitKey(1) & 0xFF == ord('q'): #Exit program when the user presses 'q'
            self.detect = False

    def run(self):
        while not rospy.is_shutdown():
            if self.detect:
                self.face_detect()

if __name__ == '__main__':
    fd = FaceDetect()
    #fd.face_detect()
    fd.run()
