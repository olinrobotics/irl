#!/usr/bin/env python
import rospy
import rospkg
import time

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

import cv2

class HandwritingRecognition:
    def nothing(x):
        pass

    def __init__(self):
        self.img = cv2.imread('test_imgs/digits.png')
        rospy.init_node('handwriting_recognition', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        self.detect = True
        cv2.namedWindow('image')

    def img_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def process_data(self):
        gray = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
        cells = [np.hsplit(row,100) for row in np.vsplit(gray,50)]
        x = np.array(cells)

        train = x[:,:].reshape(-1,400).astype(np.float32)

        k = np.arange(10)
        train_labels = np.repeat(k,500)[:,np.newaxis]

        np.savez('knn_data.npz', train=train, train_labels=train_labels)

    def train_knn(self):
        with np.load('knn_data.npz') as input_data:
            self.train_data = input_data['train']
            self.data_labels = input_data['train_labels']

        self.knn = cv2.KNearest()
        self.knn.train(self.train_data,self.data_labels)

    def find_text_area(self):
        img = self.frame
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(5,5),0)
        edges = cv2.Canny(blur,60,255)

        lines = cv2.HoughLines(edges,1,np.pi/180,140)
        if lines is not None:
            for rho,theta in lines[0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0+1000*(-b))
                y1 = int(y0+1000*(a))
                x2 = int(x0-1000*(-b))
                y2 = int(y0-1000*(a))

                cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
        cv2.imshow('image2',edges)
        self.frame = img

        # Returns a list of image ROIs (20x20) corresponding to digits found in the image
    def get_text_roi(self):
        new_frame = self.frame

        frame_gray = cv2.cvtColor(new_frame,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(frame_gray,155,255)
        # cv2.imshow('edges',edges)
        # Only looking for external contours
        out_frame = cv2.adaptiveThreshold(frame_gray,255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,15,10)
        kernel = np.ones((2,2),np.uint8)
        erode = cv2.erode(out_frame,kernel,iterations=2)
        contours,hierarchy = cv2.findContours(edges,cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_NONE)

        # Build the list of number contours and number locations
        self.number_locs = []
        number_contours = []

        # cv2.imshow('newframe',erode)
        for contour in contours:
            if not cv2.isContourConvex(contour) and len(number_contours) < 100:
                [x,y,w,h] = cv2.boundingRect(contour)
                if x >= 10 and y >= 10 and (x+w) <= (frame_gray.shape[0] - 10) and (y+h) <= (frame_gray.shape[1] - 10):
                    roi = frame_gray[y-10:y+h+10,x-10:x+w+10]

                    if len(roi) > 0: # Gets rid of weird zero-size contours
                        new_roi = cv2.resize(roi, (20,20)) # standardize contours
                        new_roi = cv2.adaptiveThreshold(new_roi,255,
                            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,17,5)
                        # dilate to make numbers more thicc
                        # new_roi = cv2.morphologyEx(new_roi, cv2.MORPH_OPEN, kernel)
                        new_roi = cv2.dilate(new_roi,kernel,iterations = 1)


                        # Record contour and contour location
                        number_contours.append(new_roi)
                        self.number_locs.append((x+w,y+h))

        # Build an image to show all number contours
        if len(number_contours) < 10:
            new_img = np.ones((20,20*len(number_contours)),np.uint8)
            y = 0
            for x in range(0,len(number_contours)):
                new_img[:,y:y+20] = number_contours[x]
                y += 20
            cv2.imshow('image3',new_img)
        data_array = np.array(number_contours)

        # Reshapes the array to be an array of 1x400 floats to fit
        # with the kNN training data
        out_data = data_array[:,:].reshape(-1,400).astype(np.float32)
        cv2.waitKey(1)
        # cv2.imshow('image',frame_gray)
        return out_data

    def process_digits(self,test_data):
        ret,result,neighbors,dist = self.knn.find_nearest(test_data,k=5)
        new_frame = self.frame
        self.res = result
        nums = []
        if result.shape[0] < 7:
            for x in self.res:
                nums.append(str(int(x.item(0))))
            for index,y in enumerate(nums):
                cv2.putText(new_frame,y,self.number_locs[index],cv2.FONT_HERSHEY_SIMPLEX, 4,(0,255,0))

    def output_image(self):
        new_frame = self.frame
        cv2.imshow('image', new_frame)
        cv2.waitKey(1)

    def run(self):
        r = rospy.Rate(10)
        time.sleep(2)
        self.process_data()
        self.train_knn()
        while not rospy.is_shutdown():
            #self.find_text_area()
            ROI = self.get_text_roi()
            #print ROI.shape
            self.process_digits(ROI)
            #cv2.imshow('image2',ROI[0].reshape(20,20).astype(float8))
            self.output_image()
            r.sleep()

if __name__ == '__main__':
    hr = HandwritingRecognition()
    hr.run()
