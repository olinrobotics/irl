'''
Purpose: to process a board of Connect4
Authors: Hannah Kolano and Kevin Zhang
Contact: hannah.kolano@students.olin.edu
Last edited: 11/13/17
'''

import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt

class DetectConnectFour:

    def __init__(self):
        '''initialize the object'''
        print("initialized")

    def run(self):
        '''initialize camera'''
        #cap = cv2.VideoCapture(0)
        #_, frame = cap.read()
        frame = cv2.imread('modelpic.png')
        framegray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) # grayscale image
        imagewidth, imageheight, __ = frame.shape

        '''define template'''
        template = cv2.imread('imagetomatch.png')
        templategray = cv2.cvtColor(template, cv2.COLOR_RGB2GRAY)

        img = cv2.medianBlur(framegray,5)
        th1 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,10)

        frame_points = self.homog_match(templategray, framegray)
        board = self.transform_to_grid(frame, np.float32(frame_points))
        self.segment_frame(board)

    def draw_basic_boxes(self, contours, frame):
        for cnt in contours:
            x,y,w,h = cv2.boundingRect(cnt)
            if(w*h) >= 2000 and w*h < imagewidth*imageheight:
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)

    def draw_contours(self, thresholdedpic, origpic):
        #TODO:
        ''' check out https://docs.opencv.org/3.3.0/d9/d61/tutorial_py_morphological_ops.html'''
        __, contours, __ = cv2.findContours(thresholdedpic,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(origpic, contours, -1, (0,255,0), 2)

    def homog_match(self, trainimage, frame):
        MIN_MATCH_COUNT = 5
        sift = cv2.xfeatures2d.SIFT_create()
        kp1, des1 = sift.detectAndCompute(trainimage,None)
        kp2, des2 = sift.detectAndCompute(frame,None)

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks = 50)

        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, des2, k=2)

        good = []
        for m, n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        if len(good)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            h,w = trainimage.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)

            frame = cv2.polylines(frame,[np.int32(dst)],True,255,3, cv2.LINE_AA)

        else:
            print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
            matchesMask = None

        '''visualize matches'''
        #draw_params = dict(matchColor = (0, 255,0), singlePointColor = None, matchesMask = matchesMask, flags = 2)
        #img3 = cv2.drawMatches(trainimage, kp1, frame, kp2, good, None, **draw_params)
        #plt.imshow(img3, 'gray'), plt.show()
        return dst

    def transform_to_grid(self, frame, points):
        zoomed_in = np.float32([[0,0],[0,300],[400,300],[400,0]])
        N = cv2.getPerspectiveTransform(points, zoomed_in)
        dst2 = cv2.warpPerspective(frame,N,(400,300))
        return dst2

    def segment_frame(self, img):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        # noise removal
        kernel = np.ones((3,3),np.uint8)
        opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)

        # sure background area
        sure_bg = cv2.dilate(opening,kernel,iterations=3)

        # Finding sure foreground area
        dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
        ret, sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)

        # Finding unknown region
        sure_fg = np.uint8(sure_fg)
        unknown = cv2.subtract(sure_bg,sure_fg)
        self.show_image(sure_fg)
        # Marker labelling
        ret, markers = cv2.connectedComponents(sure_bg)

        # Add one to all labels so that sure background is not 0, but 1
        markers = markers+1

        # Now, mark the region of unknown with zero
        markers[unknown==255] = 0

        markers = cv2.watershed(img,markers)
        img[markers == -1] = [255,0,0]
        self.show_image(img)

    def show_image(self, frame):
        while(True):
            cv2.imshow('frame', frame)
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                cv2.destroyAllWindows()
                break
            elif k == ord('s'):
                cv2.imwrite('thisthing.png', frame)

    #TODO:
    '''Figure out what the current layout is like'''
    '''image segmentation
    https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_watershed/py_watershed.html#watershed
    '''

    #TODO:
    '''Format to send to the machine learning part'''

if __name__ =='__main__':
    detect = DetectConnectFour()
    detect.run()
