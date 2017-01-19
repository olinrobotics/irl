#!/usr/bin/env python
import rospkg
import rospy

import cv2
import sys
import logging as log
import datetime as dt

from time import sleep
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class FaceDetect:

    def __init__(self):

        #detection boolean
        self.detect = True

        #initializes frame
        self.frame = None

        #initializes ros node for face detect, pubilishes to face location
        rospy.init_node('face_detect', anonymous=True)
        self.pub = rospy.Publisher('/face_location', String, queue_size=10)

        #definines cascade path
        # cascPath = sys.argv[1]
        # self.face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        self.face_cascade = cv2.CascadeClassifier(PACKAGE_PATH + '/params/haarcascade_frontalface_alt.xml')
        self.smile_cascade = cv2.CascadeClassifier(PACKAGE_PATH + '/params/haarcascade_smile.xml')

        #CvBridge to usb_cam, subscribes to usb cam
        self.bridge = CvBridge()
        rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)

        #
        self.pub = rospy.Publisher('/smile_detected', String, queue_size = 10)

        print "FaceDetect is running"

    #converts ros message to numpy
    def img_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            h, w = self.frame.shape[:2]
        except CvBridgeError as e:
            print(e)

    #face_detect method
    def face_detect(self):
        if self.frame == None:
            return

        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.2,
            minNeighbors=5,
            minSize=(20, 20),
            flags=cv2.cv.CV_HAAR_SCALE_IMAGE
        )

        face = None
        #draws rectangle around face

        #biggest face
        biggestArea = 0;
        smile_msg = 'False'
        msg = ''
        for (x, y, w, h) in faces:
            cv2.rectangle(self.frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            #crops face from image
            face = gray[y:y+h,x:x+w]

            smile = self.smile_cascade.detectMultiScale(
                face,
                scaleFactor= 1.7,
                minNeighbors=22,
                minSize=(25, 25),
                flags=cv2.cv.CV_HAAR_SCALE_IMAGE
                )

            #find biggest face
            area = w*h;
            center_x = x + w/2
            center_y = y + h/2

            if len(smile) > 0:
                smile_msg = 'True'

            if area > biggestArea:
                area = biggestArea
                msg = str(center_x) + ',' + str(center_y) + ':' + smile_msg

            for (x, y, w, h) in smile:
                print "Found", len(smile), "smiles!"
                cv2.rectangle(self.frame, (x, y), (x+w, y+h), (255, 0, 0), 1)

        if msg != '':
            self.pub.publish(msg)

        # #displays resulting frame
        cv2.imshow('Video', self.frame)
        if face != None:
            cv2.imshow('Face', face)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.detect = False
        pass


    def run(self):
        while not rospy.is_shutdown():
            if self.detect:
                self.face_detect()

if __name__ == '__main__':
    fd = FaceDetect()
    #fd.face_detect()
    fd.run()
