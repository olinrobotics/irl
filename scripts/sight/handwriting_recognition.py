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
from os import listdir
from os.path import isfile, join

import cv2

class HandwritingRecognition:
    def nothing(x):
        pass

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
        # print os.getcwd()

        # Defines vars for keeping track of new words
        self.last_word = ''
        self.last_time = time.time()
        self.curr_data = ''
        self.found_word = False

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
        path = 'char_data/'
        files = listdir(path)

        train_data = np.float32(np.zeros((0,64)))
        responses = np.float32(np.zeros((0,1)))
        for f in files: # Loads the .pngs for the training data for each letter
            file_path = path + f
            letter = f.split('.',1)[0]
            train_img = cv2.imread(file_path)
            cells_data = []
            # Processes the training data into a usable format
            gray = cv2.cvtColor(train_img,cv2.COLOR_BGR2GRAY)
            cells = [np.hsplit(row,10) for row in np.vsplit(gray,10)]
            # deskewed = [map(self.deskew,row) for row in cells]
            hogdata = [map(self.hog,row) for row in cells]

            train_data = np.concatenate((train_data,np.float32(hogdata).reshape(-1,64)),axis=0)
            # Builds the labels for the training data
            responses = np.concatenate((responses,np.float32(np.repeat([ord(letter)],100)[:,np.newaxis])),axis=0)
        np.savez('svm_data.npz',train=train_data,train_labels=responses)

        # Train the SVM neural network to recognize characters
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
                [x,y,w,h] = cv2.boundingRect(contour)
                if  bound < x < (frame_gray.shape[1] - bound) and bound < y < (frame_gray.shape[0] - bound) and (x+w) <= (frame_gray.shape[1] - bound) and (y+h) <= (frame_gray.shape[0] - bound):
                    roi = frame_gray[y-bound:y+h+bound,x-bound:x+w+bound]

                    if len(roi) > 0: # Gets rid of weird zero-size contours
                        new_roi = cv2.adaptiveThreshold(roi,255,
                            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,35,7)
                        new_roi = cv2.morphologyEx(new_roi,cv2.MORPH_OPEN,kernel,iterations=2) # Embiggen numbers
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
            # Prepares input data for processing
            reshape_data = np.float32([char.HOG for char in test_data]).reshape(-1,64)

            result = self.SVM.predict_all(reshape_data)
            res_data = []
            for x in result:
                res_data.append(int(x.item(0)))
            for idx,roi in enumerate(self.chars):
                roi.result = res_data[idx]
                # Formats results from SVM processing
                cv2.putText(self.frame,chr(int(roi.result)),(roi.x,roi.y+roi.h) \
                            ,cv2.FONT_HERSHEY_SIMPLEX, 4,(0,255,0))
            self.detect_new_word(test_data)

    def detect_new_word(self,char_list):
        char_list.sort(key = lambda roi: roi.x)
        word = ''.join([chr(item.result) for item in char_list])
        if word == self.curr_data:
            if time.time() - self.last_time > 2 and self.found_word == False:
                self.last_word = word
                self.found_word = True
                print word
        else:
            self.last_time = time.time()
            self.curr_data = word
            self.found_word = False



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
            #print ' '.join(words)
        # print ''
        # for word in lines:
        #     print ' '.join([str(x.result) for x in word])

    # Update the current frame
    def update_frame(self):
        self.frame = self.curr_frame

    # display the current image
=======
        cv2.createTrackbar('K','image',1,255,self.nothing)
        cv2.setTrackbarPos('K','image',5)

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
        # out_frame = cv2.adaptiveThreshold(frame_gray,255,
        #     cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,15,10)
        kernel = np.ones((2,2),np.uint8)
        # erode = cv2.erode(out_frame,kernel,iterations=2)
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
        out_data = data_array[:,:].reshape(-1,400).astype(np.float32)
        cv2.waitKey(1)
        # cv2.imshow('image',frame_gray)
        return out_data

    def process_digits(self,test_data):
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

>>>>>>> For real things work this time.
    def output_image(self):
        new_frame = self.frame
        cv2.imshow('image', new_frame)
        cv2.waitKey(1)

<<<<<<< HEAD
    def run(self):
        r = rospy.Rate(10)
        time.sleep(2)
        self.process_data_svm()
        self.train_svm()
        self.test_ocr() # print accuracy of current OCR
        while not rospy.is_shutdown():
            e1 = cv2.getTickCount()
            self.update_frame()
            self.chars = self.get_text_roi()
            self.process_digits(self.chars)
            #self.find_words(self.chars)
            self.output_image()
            e2 = cv2.getTickCount()
            # print (e2-e1)/cv2.getTickFrequency()
=======
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
        while not rospy.is_shutdown():
            #self.find_text_area()
            ROI = self.get_text_roi()
            #print ROI.shape
            self.process_digits(ROI)
            #cv2.imshow('image2',ROI[0].reshape(20,20).astype(float8))
            self.output_image()
>>>>>>> For real things work this time.
            r.sleep()

if __name__ == '__main__':
    hr = HandwritingRecognition()
    hr.run()
