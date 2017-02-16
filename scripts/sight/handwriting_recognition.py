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
import img_processing as Process #Library of image processing functions in edwin/scripts/sight
import csv

class HandwritingRecognition: # HR object
    def nothing(x): # empty callback function to pass as parameter
        pass

    def __init__(self, init_param=False):
        '''
            DESC: runs once when program starts, sets up initial state, vars, etc.
            ARGS:
            self - reference to current HR object
            init_param - Boolean providing alternate setup initialization
            RETURN: none
            '''
        if init_param:
            # init_param allows us to instantiate the HR object in different contexts, i.e in WritingDemo.py
            pass
        else:
            rospy.init_node('handwriting_recognition', anonymous=True) # makes self into node
            rospy.Subscriber('usb_cam/image_raw', Image, self.img_callback) # subscribes to cam feed

            self.bridge = CvBridge() # converts image types to use with OpenCV

        # Builds path to find picture of numbers
        rospack = rospkg.RosPack()
        self.PARAMS_PATH = rospack.get_path('edwin')
        self.img = cv2.imread(self.PARAMS_PATH + '/params/test_imgs/digits.png')

        self.pub = rospy.Publisher('word_publish',String,queue_size=10) # publishes found words

        self.detect = True
        self.frame = None

        # Builds window to view and control output
        cv2.namedWindow('image')
        cv2.createTrackbar('X','image',0,255,self.nothing)
        cv2.setTrackbarPos('X','image',255)
        cv2.createTrackbar('Y','image',0,255,self.nothing)
        cv2.setTrackbarPos('Y','image',7)
        self.test_data = np.zeros((200,200),np.uint8)
        self.test_filled = 0
        # print os.getcwd()

        # Define vars for keeping track of new words
        self.last_word = ''
        self.last_time = time.time()
        self.curr_data = ''
        self.found_word = False

    def test_ocr(self):
        '''
            DESC: Compares Support Vector Machine(SVM)-guessed set against training data
            ARGS: self - reference to current HR object
            RETURNS: none
            SHOWS: prints accuracy of the SVM
            '''
        labels = []
        # Prepares the labels from the text file
        with open(self.PARAMS_PATH + '/params/train_data.txt', 'r') as test_data:
            open_reader = csv.reader(test_data)
            for line in open_reader:
                labels.append(ord(line[0])) # adds label to list labels per line

        test_img = cv2.imread(self.PARAMS_PATH + '/params/test_data.png') # reads image, saves as test_img
        cells = [np.hsplit(row,10) for row in np.vsplit(test_img,10)] #
        hogdata = [map(Process.hog,row) for row in cells]
        test_data = np.float32(hogdata).reshape(-1,64)
        result = [int(res) for res in self.SVM.predict_all(test_data)]
        # Compares predicted with actual results
        matches = [i for i, j in zip(result, labels) if i == j]
        accuracy = len(matches)
        print 'Accuracy: ', accuracy


    def img_callback(self, data):
        try:
            self.curr_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


    def process_data_svm(self):
        '''
            DESC:
            ARGS:
            RETURNS:
            '''
        path =  self.PARAMS_PATH + '/params/char_data/'
        files = listdir(path) #Lists files in path folder

        train_data = np.float32(np.zeros((0,64)))
        responses = np.float32(np.zeros((0,1)))
        for f in files: # Loads the .pngs for the training data for each symbol
            file_path = path + f
            code = f.split('.',1)[0]
            train_img = cv2.imread(file_path)
            cells_data = []
            # Processes the training data into a usable format
            gray = cv2.cvtColor(train_img,cv2.COLOR_BGR2GRAY)
            cells = [np.hsplit(row,10) for row in np.vsplit(gray,10)]
            # deskewed = [map(self.deskew,row) for row in cells]
            hogdata = [map(Process.hog,row) for row in cells]

            train_data = np.concatenate((train_data,np.float32(hogdata).reshape(-1,64)),axis=0)
            # Builds the labels for the training data
            responses = np.concatenate((responses,np.float32(np.repeat([self.decode_file(code)],100)[:,np.newaxis])),axis=0)
        np.savez(self.PARAMS_PATH + '/params/svm_data.npz',train=train_data,train_labels=responses)

        # Train the SVM neural network to recognize characters

    def decode_file(self, code):
        '''
            DESC: Translates file name (part before .png) into Unicode index
            for the character represented by the file name. Used to get around
            the inability to name files with symbols such as /,-,
            *, etc.
            ARGS:
            self - HandWriting object - reference to current object
            code - string - file name up to, but not including, .png
            RTRN: string directly representing character
            '''
        if (code == 'mlt'):
            return ord('*')
        elif (code == 'dvd'):
            return ord('/')
        elif (code == 'pls'):
            return ord('+')
        elif (code == 'mns'):
            return ord('-')
        elif (len(code) == 1):
            return ord(code)


    def train_svm(self):
        svm_params = dict(kernel_type = cv2.SVM_LINEAR, svm_type = \
                            cv2.SVM_NU_SVC, nu=.105)
                            # gamma = 5.383

        with np.load(self.PARAMS_PATH + '/params/svm_data.npz') as input_data:
            train_data = input_data['train']
            data_labels = input_data['train_labels']

        self.SVM = cv2.SVM()
        self.SVM.train(train_data,data_labels,params=svm_params)

        # Returns a list of image Regions Of Interest(ROIs) (20x20) corresponding to digits found in the image


    def process_digits(self,test_data,detect_words = False):
        if len(test_data) != 0:
            # Prepares input data for processing
            reshape_data = np.float32([char.HOG for char in test_data]).reshape(-1,64)

            results = self.SVM.predict_all(reshape_data)
            res_data = []
            for x in results:
                res_data.append(int(x.item(0)))
            for idx,roi in enumerate(self.chars):
                roi.result = res_data[idx]
                # Formats results from SVM processing

            # Resolves contours that could be an 'i' or a 'j'
            # 0 = dots, 1 = lines
            lines = [val for val in self.chars if chr(val.result) == '1']
            dots = [res for res in self.chars if chr(res.result) == '0']
            self.chars[:] = [val for val in self.chars if chr(val.result) != '1' \
                and chr(val.result) != '0']
            i_conts = self.resolve_letters(dots,lines)
            if len(i_conts) > 0:
                self.chars.extend(i_conts)
            if self.frame is not None:
                for roi in self.chars:
                    cv2.putText(self.frame,chr(int(roi.result)),(roi.x,roi.y+roi.h) \
                                ,cv2.FONT_HERSHEY_SIMPLEX, 4,(0,255,0))
            if detect_words:
                self.detect_new_word(test_data)
            else:
                return test_data

    # Resolves ambiguous letters (i,l)
    def resolve_letters(self,dot_contours,line_contours):
        i_contours = []
        l_contours = []
        for dot in dot_contours:
            for line in line_contours:
                # If this is true, a letter 'i' has been found
                if abs(line.x - dot.x) < 50 and line.y > dot.y:
                    dot.result = ord('i')
                    dot.h += line.h
                    dot.y = line.y
                    # Remove the dot-line pair from our lists
                    line_contours[:] = [val for val in line_contours if val is not line]
                    dot_contours[:] = [val for val in dot_contours if val is not dot]
                    i_contours.append(dot) # Add the new character to the list
        for line in line_contours: # All remaining lines must be 'l' chars
            line.result = ord('l')
        try:
            line_contours.extend(i_contours)
        except AttributeError:
            pass
        return line_contours


    def detect_new_word(self,char_list):
        char_list.sort(key = lambda roi: roi.x) # Sort characters by x pos
        word = ''.join([chr(item.result) for item in char_list]) # Form a word
        if word == self.curr_data: # If the current and prev words match
            # The word must remain consistent for 2 seconds
            if time.time() - self.last_time > 2 and self.found_word == False:
                # We found a new word
                self.last_word = word
                self.found_word = True
                self.pub.publish(word)
                return word
        else: # A new word is found, reset the timer
            self.last_time = time.time()
            self.curr_data = word
            self.found_word = False
            return None


    ''' function find_words()
        PURPOSE:
        ARGUMENTS:
        self: object that refers to the current HandwritingRecognition object
        char_list:
        RETURNS:
        '''
    # This is incomplete
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

    ''' function update_frame
        PURPOSE: update frame variable for use
        ARGUMENTS:
        self: object that refers to the current HandwritingRecognition object
        RETURNS: None
        SHOWS: None
        '''
    def update_frame(self):
        self.frame = self.curr_frame

    ''' function output_image()
        PURPOSE: Display current frame
        ARGUMENTS:
        self: object that refers to the current HandwritingRecognition object
        RETURNS: None
        SHOWS: Displays current frame
        '''
    def output_image(self):
        new_frame = self.frame
        cv2.imshow('image', new_frame)
        cv2.waitKey(1)


    def get_image_text(self, frame):
        '''
            DESC: Get the words out of an image
            ARGS:
                self - object - current HR object
                frame - image - current image to analyze
            RETURNS: string representing text found in frame
            '''
        self.chars = Process.get_text_roi(frame,show_window=False)
        self.chars = self.process_digits(self.chars)

        # If chars exists,
        if self.chars:
            self.chars.sort(key = lambda roi: roi.x)
            word = ''.join([chr(item.result) for item in self.chars])
            return word


    def run(self):
        r = rospy.Rate(10)
        time.sleep(2)
        self.process_data_svm()
        self.train_svm()
        self.test_ocr() # print accuracy of current OCR
        while not rospy.is_shutdown():
            e1 = cv2.getTickCount()
            self.update_frame()
            # out_image = Process.get_paper_region(self.frame)
            self.chars = Process.get_text_roi(self.frame)
            self.process_digits(self.chars,detect_words=True)
            #self.find_words(self.chars)
            self.output_image()
            e2 = cv2.getTickCount()
            # print (e2-e1)/cv2.getTickFrequency()
            r.sleep()

if __name__ == '__main__':
    hr = HandwritingRecognition()
    hr.run()
