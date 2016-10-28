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

class HandwritingRecognition:

    def __init__(self):
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
        deskewed = [map(self.deskew,row) for row in cells]
        # print deskewed[0][0]
        hogdata = [map(self.hog,row) for row in deskewed]
        test_data = np.float32(hogdata).reshape(-1,64)
        labels = np.array([[9],[8],[5],[6],[2],[3],[7],[8],[4],[5],
                           [1],[9],[7],[8],[4],[5],[1],[3],[2],[9],
                           [7],[8],[6],[5],[2],[3],[9],[8],[5],[6],
                           [7],[8],[5],[7],[8],[5],[7],[8],[5],[9],
                           [7],[8],[6],[5],[2],[3],[9],[7],[8],[5],
                           [2],[3],[7],[8],[5],[5],[1],[2],[7],[8],
                           [4],[5],[7],[8],[5],[9],[8],[7],[6],[5],
                           [2],[3],[9],[8],[7],[6],[5],[2],[3],[7],
                           [8],[4],[5],[7],[8],[4],[5],[8],[7],[8],
                           [4],[5],[1],[2],[7],[8],[4],[5],[1],[2]])
        result = self.SVM.predict_all(test_data)
        matches = result==labels
        correct = np.count_nonzero(matches)
        accuracy = correct*100.0/result.size
        print 'Accuracy: ', accuracy

    def img_callback(self, data):
        try:
            self.curr_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def process_data_svm(self):
        cells_data = []
        gray = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
        cells = [np.hsplit(row,100) for row in np.vsplit(gray,50)]
        # print cells[0][0]
        deskewed = [map(self.deskew,row) for row in cells]
        # print deskewed[0][0]
        hogdata = [map(self.hog,row) for row in deskewed]
        train_data = np.float32(hogdata).reshape(-1,64)
        responses = np.float32(np.repeat(np.arange(10),500))[:,np.newaxis]
        np.savez('svm_data.npz',train=train_data,train_labels=responses)

    def train_svm(self):
        svm_params = dict(kernel_type = cv2.SVM_LINEAR, svm_type = \
                            cv2.SVM_C_SVC, C=2.67, gamma=5.383)

        with np.load('svm_data.npz') as input_data:
            train_data = input_data['train']
            data_labels = input_data['train_labels']

        self.SVM = cv2.SVM()
        self.SVM.train(train_data,data_labels,params=svm_params)

        # Returns a list of image ROIs (20x20) corresponding to digits found in the image
    def get_text_roi(self):
        chars = []
        bound = 5
        kernel = np.ones((2,2),np.uint8)

        frame_gray = cv2.cvtColor(self.frame,cv2.COLOR_BGR2GRAY)
        frame_gray = cv2.GaussianBlur(frame_gray, (5,5),0) # Gaussian blur to remove noise

        # Adaptive threshold to find numbers on paper
        thresh = cv2.adaptiveThreshold(frame_gray,255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,35,7)
        thresh = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel,iterations=3)
        # cv2.imshow('thresh',thresh)

        contours,hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(self.frame,contours,-1,(255,0,0),2)
        # Build the list of number contours and number locations

        if len(contours) < 35:
            for ind,contour in enumerate(contours):
                # print hierarchy[0][ind]
                [x,y,w,h] = cv2.boundingRect(contour)
                if  bound < x < (frame_gray.shape[1] - bound) and bound < y < (frame_gray.shape[0] - bound) and (x+w) <= (frame_gray.shape[1] - bound) and (y+h) <= (frame_gray.shape[0] - bound):
                    roi = frame_gray[y-bound:y+h+bound,x-bound:x+w+bound]

                    if len(roi) > 0: # Gets rid of weird zero-size contours
                        new_roi = cv2.adaptiveThreshold(roi,255,
                            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,31,3)
                        new_roi = cv2.dilate(new_roi,kernel,iterations=2) # Embiggen numbers
                        new_roi = cv2.resize(new_roi, (20,20), interpolation=cv2.INTER_AREA) # standardize contours
                        deskewed = self.deskew(new_roi)
                        # Record contour and contour location, and filter out internal contours
                        if hierarchy[0][ind][3] == -1:
                            cont = Character(deskewed,(x,y,w,h), self.hog(deskewed))
                            chars.append(cont)

        # Build an image to show all number contours
        num_len = len(chars)
        # print '# of contours: ', num_len
        if num_len < 35 and num_len > 0:
            new_img = np.ones((20,20*num_len),np.uint8)
            y = 0
            for x in chars:
                new_img[:,y:y+20] = x.img
                y += 20
            cv2.imshow('image3',new_img)
        return chars
        # Reshapes the array to be an array of 1x400 floats to fit
        # with the kNN training data

        # Deskews a 20x20 character image
    def deskew(self,img):
        SZ = 20
        affine_flags = cv2.WARP_INVERSE_MAP|cv2.INTER_LINEAR
        m = cv2.moments(img)
        if abs(m['mu02']) < 1e-2:
            return img.copy()
        # print m
        skew = m['mu11']/m['mu02']
        M = np.float32([[1,skew,-0.5*SZ*skew], [0,1,0]])
        img = cv2.warpAffine(img,M,(SZ,SZ),flags=affine_flags)
        return img

        # Retursn the HOG for a given imagej
    def hog(self,img):
        bin_n = 16
        gx = cv2.Sobel(img, cv2.CV_32F,1,0) # x gradient
        gy = cv2.Sobel(img, cv2.CV_32F,0,1) # y gradient
        mag,ang = cv2.cartToPolar(gx,gy) # Polar gradients
        bins = np.int32(bin_n*ang/(2*np.pi)) # creating binvalues
        bin_cells = bins[:10,:10],bins[10:,:10],bins[:10,10:],bins[10:,10:]
        mag_cells = mag[:10,:10],mag[10:,:10],mag[:10,10:],mag[10:,10:]
        hists = [np.bincount(b.ravel(), m.ravel(), bin_n) for b, m in zip(bin_cells, mag_cells)]
        hist = np.hstack(hists) # hist is a 64-bit vector
        return hist


    def process_digits(self,test_data):
        if len(test_data) != 0:
            # print 'test data length: ', len(test_data)
            # k_val = cv2.getTrackbarPos('K','image')
            # ret,result,neighbors,dist = self.knn.find_nearest(test_data,k=2)
            reshape_data = np.float32([char.HOG for char in test_data]).reshape(-1,64)

            result = self.SVM.predict_all(reshape_data)
            res_data = []
            for x in result:
                res_data.append(int(x.item(0)))
            for idx,roi in enumerate(self.chars):
                roi.result = res_data[idx]
                cv2.putText(self.frame,str(roi.result),(roi.x+roi.w,roi.y+roi.h) \
                            ,cv2.FONT_HERSHEY_SIMPLEX, 4,(0,255,0))

    def find_words(self,char_list):
        # Find lines (this is terrible - refactor)
        line_space = []
        char_list.sort(key = lambda roi: roi.y)
        for ind in range(1,len(char_list)):
            next_y = char_list[ind].y
            curr_y = char_list[ind-1].y
            line_space.append(char_list[ind].y - char_list[ind-1].y)
        z_vals = stats.zscore(line_space)

        line_index = [0]
        for idx,val in enumerate(z_vals):
            if val > 1:
                line_index.append(idx+1)
        line_index.append(len(char_list))
        # print all_chars
        # print word_index
        lines = []
        for idx in range(0,len(line_index)-1):
            lines.append(char_list[line_index[idx]:line_index[idx+1]])

        # Find words in lines(this is terrible - refactor)
        for line in lines:
            space_size = []
            line.sort(key = lambda roi: roi.x)
            for ind in range(1,len(line)):
                next_x = char_list[ind+1].x
                curr_x = char_list[ind].x + char_list[ind].h
                space_size.append(char_list[ind].x - char_list[ind-1].x)
            z_vals = stats.zscore(space_size)

            word_index = [0]
            for idx,val in enumerate(z_vals):
                if val > 1:
                    word_index.append(idx+1)
            word_index.append(len(line))

            words = []
            for idx in range(0,len(word_index)-1):
                words.append(''.join([str(x.result) for x in line[word_index[idx]:word_index[idx+1]]]))
            print ' '.join(words)
        print ''
        # for word in lines:
        #     print ' '.join([str(x.result) for x in word])






    # Update the current frame
    def update_frame(self):
        self.frame = self.curr_frame

    # display the current image
    def output_image(self):
        new_frame = self.frame
        cv2.imshow('image', new_frame)
        cv2.waitKey(1)

    def run(self):
        r = rospy.Rate(10)
        time.sleep(2)
        self.process_data_svm()
        self.train_svm()
        self.test_ocr() # print accuracy of current OCR
        while not rospy.is_shutdown():
            self.update_frame()
            self.chars = self.get_text_roi()
            self.process_digits(self.chars)
            #self.find_words(self.chars)
            self.output_image()
            r.sleep()

if __name__ == '__main__':
    hr = HandwritingRecognition()
    hr.run()
