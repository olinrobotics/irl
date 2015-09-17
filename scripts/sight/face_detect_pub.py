#!/usr/bin/python

import cv2
import time
import Image
import roslib; roslib.load_manifest('jimmy')
import rospy
from std_msgs.msg import String
import numpy as np

class Face:
    def __init__(self,x,y,w,h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.midx = x + w/2
        self.midy = y + h/2

def largestFaceIndex(faces):
    largestIndex = 0;
    largest = 0
    for index, face in enumerate(faces):
        if face.w > largest:
            largest = face.w
            largestIndex = index
    return largestIndex

def publisher():
    rospy.init_node("face_location",anonymous = True)
    
    cap = cv2.VideoCapture(1)
    cap.set(3, 480)
    cap.set(4, 360)

    face_cascade = cv2.CascadeClassifier('/home/skelly1/opencv-2.4.10/data/haarcascades/haarcascade_frontalface_alt.xml')

    pub = rospy.Publisher('face_location', String, queue_size = 10)
    while not rospy.is_shutdown():
        ret, frame = cap.read()

        faces = face_cascade.detectMultiScale(frame, scaleFactor=1.2, minSize=(20,20))
        if len(faces) > 0:
            faceObjects = []
            for (x, y, w, h) in faces:
                faceObjects.append(Face(x,y,w,h))

            for (x,y,w,h) in faces:
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255))

            face = faceObjects[largestFaceIndex(faceObjects)]
            print "largest face at:"+str(face.midx)+", "+str(face.midy)
            pub.publish(str(face.midx)+","+str(face.midy))
        else:
            print "no face detected: returning (x,y) = (300,250)"
            pub.publish(str(300)+","+str(250))
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    publisher()
