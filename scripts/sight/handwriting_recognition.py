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
        cv2.createTrackbar('K','image',1,255,self.nothing)
        cv2.setTrackbarPos('K','image',5)
        cv2.setMouseCallback('image',self.fill_test_data)
        self.test_data = np.zeros((200,200),np.uint8)
        self.test_filled = 0

    def fill_test_data(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            for contour in self.numbers:
                if self.test_filled < 100:
                    x_index = (self.test_filled%10)*20
                    y_index = (self.test_filled // 10)*20
                    self.test_data[y_index:y_index+20,x_index:x_index+20] = contour
                    self.test_filled += 1
        elif event == cv2.EVENT_RBUTTONDOWN:
            cv2.imwrite('test_data.png',self.test_data)

    def test_ocr(self):
        data_img = cv2.imread('test_data.png')
        gray = cv2.cvtColor(data_img,cv2.COLOR_BGR2GRAY)
        cells = [np.hsplit(row,10) for row in np.vsplit(gray,10)]
        x = np.array(cells)
        test = x[:,:].reshape(-1,400).astype(np.float32)
        labels = np.array([[8],[9],[5],[3],[4],[8],[7],[3],[1],[2],
                           [8],[7],[9],[2],[3],[4],[8],[9],[0],[3],
                           [4],[5],[5],[4],[1],[2],[7],[9],[7],[6],
                           [5],[2],[3],[1],[5],[2],[7],[8],[5],[7],
                           [8],[4],[5],[7],[8],[4],[5],[1],[2],[6],
                           [5],[4],[2],[2],[1],[9],[8],[8],[5],[4],
                           [8],[7],[7],[4],[4],[3],[2],[3],[4],[1],
                           [5],[2],[9],[7],[8],[8],[7],[5],[4],[1],
                           [2],[8],[7],[5],[4],[1],[7],[7],[7],[9],
                           [8],[8],[7],[8],[7],[1],[6],[4],[8],[6]])
        ret,result,neighbors,dist = self.knn.find_nearest(test,k=2)
        matches = result==labels
        correct = np.count_nonzero(matches)
        accuracy = correct*100.0/result.size
        print 'Accuracy: ', accuracy

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
        bound = 15
        new_frame = self.frame
        kernel = np.ones((2,2),np.uint8)


        frame_gray = cv2.cvtColor(new_frame,cv2.COLOR_BGR2GRAY)
        frame_gray = cv2.GaussianBlur(frame_gray, (5,5),0) # Gaussian blur to remove noise

        # Adaptive threshold to find numbers on paper
        thresh = cv2.adaptiveThreshold(frame_gray,255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,31,3)
        thresh = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel,iterations=3)


        contours,hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_NONE)
        # Build the list of number contours and number locations


        self.number_locs = []
        number_contours = []
        if len(number_contours) < 100:
            for ind,contour in enumerate(contours):
                # print hierarchy[0][ind]
                [x,y,w,h] = cv2.boundingRect(contour)
                if  bound < x < (frame_gray.shape[0] - bound) and bound < y < (frame_gray.shape[1] - bound) and (x+w) <= (frame_gray.shape[0] - bound) and (y+h) <= (frame_gray.shape[1] - bound):
                    roi = frame_gray[y-bound:y+h+bound,x-bound:x+w+bound]

                    if len(roi) > 0: # Gets rid of weird zero-size contours
                        new_roi = cv2.adaptiveThreshold(roi,255,
                            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,31,3)
                        new_roi = cv2.dilate(new_roi,kernel,iterations=2) # Embiggen numbers
                        new_roi = cv2.resize(new_roi, (20,20), interpolation=cv2.INTER_AREA) # standardize contours

                        # Record contour and contour location, and filter out internal contours
                        if hierarchy[0][ind][3] == -1:
                            number_contours.append(new_roi)
                            self.number_locs.append((x+w,y+h))

        # Build an image to show all number contours
        self.numbers = number_contours
        if len(number_contours) < 10 and len(number_contours) > 0:
            new_img = np.ones((20,20*len(number_contours)),np.uint8)
            y = 0
            for x in range(0,len(number_contours)):
                new_img[:,y:y+20] = number_contours[x]
                y += 20
            cv2.imshow('image3',new_img)
        data_array = np.array(number_contours)

        # Reshapes the array to be an array of 1x400 floats to fit
        # with the kNN training data
        try:
            out_data = data_array[:,:].reshape(-1,400).astype(np.float32)
            return out_data
        except IndexError:
            print 'Something went wrong.'
        cv2.waitKey(1)
        return None

    def process_digits(self,test_data):
        if test_data is not None:
            k_val = cv2.getTrackbarPos('K','image')
            ret,result,neighbors,dist = self.knn.find_nearest(test_data,k=2)
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
        if self.test_data is not None:
            cv2.imshow('test', self.test_data)
        cv2.waitKey(1)

    def deskew(self,img):
        m = cv2.moments(img)
        if abs(m['mu02']) < 1e-2:
            return img.copy()
        skew = m['null']/m['mu02']
        M = np.float32([[1,skew,-0.5*SZ*skew], [0,1,0]])
        img = cv2.warpAffine(img,M,(SZ,SZ),flags=affine_flags)

    def run(self):
        r = rospy.Rate(10)
        time.sleep(2)
        self.process_data()
        self.train_knn()
        self.test_ocr()
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
