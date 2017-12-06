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

    def __init__(self, numcols, numrows):
        '''initialize the object'''
        self.numcols = numcols
        self.numrows = numrows
        self.width, self.height = numcols*200, numrows*200
        print("initialized")

    def run(self):
        '''initialize camera'''
        #cap = cv2.VideoCapture(0)
        #_, frame = cap.read()
        frame = cv2.imread('View3.jpg')
        frame = cv2.resize(frame,None,fx=.2, fy=.2, interpolation = cv2.INTER_AREA)
        framegray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) # grayscale image
        #ret, threshCAM = cv2.threshold(framegray, 25, 255, 0)

        '''define template'''
        template = cv2.imread('tryingagain.jpg')
        template = cv2.resize(template, None, fx=.2, fy=.2, interpolation= cv2.INTER_AREA)
        templategray = cv2.cvtColor(template, cv2.COLOR_RGB2GRAY)

        '''find the frames'''
        viewsilhouette = self.extract_black(frame)
        templatesilhouette = self.extract_black(template)

        '''find the major contour, reduce the field of view'''
        contours = self.draw_contours(viewsilhouette, frame)
        x, y, w, h = self.draw_basic_boxes(contours, frame)
        board = self.transform_to_grid(frame, np.float32([[x,y],[x,y+h],[x+w,y+h],[x+w,y]]))

        '''warp transform to actual grid'''
        actual_corners = self.detect_corners(board)
        board = self.transform_to_grid(board, actual_corners)
        self.show_image(board, 'after grid')

    def draw_basic_boxes(self, contours, frame):
        imagewidth, imageheight, __ = frame.shape
        areas = []
        i = 0
        for cnt in contours:
            x,y,w,h = cv2.boundingRect(cnt)
            if(w*h) >= 10000 and w*h < imagewidth*imageheight-100:
                areas.append((i, w*h))
            i += 1
        areas = sorted(areas, key=lambda x: x[1])
        biggest_box = areas[-1]
        x,y,w,h = cv2.boundingRect(contours[biggest_box[0]])
        #cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
        return x-10, y-10, w+20, h+20

    def draw_contours(self, thresholdedpic, origpic):
        #TODO:
        ''' check out https://docs.opencv.org/3.3.0/d9/d61/tutorial_py_morphological_ops.html'''
        __, contours, __ = cv2.findContours(thresholdedpic,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(origpic, contours, -1, (0,255,0), 2)
        return contours

    def extract_black(self, frame):
        # convert BGR to HSV
        ret, frame2 = cv2.threshold(frame, 50, 255, cv2.THRESH_BINARY_INV)
        hsv = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        lower_ref = np.array([0, 0, 200])
        upper_ref = np.array([179, 100, 255])

        # Threshold the SSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_ref, upper_ref)
        mask = 255 - mask
        return mask

    def homog_match(self, trainimage, frame):
        MIN_MATCH_COUNT = 10
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
            if m.distance < 1.5*n.distance:
                good.append(m)

        if len(good)>MIN_MATCH_COUNT:
            print "Found enough matches"
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            h,w = trainimage.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)

            #cv2.polylines(frame,[np.int32(dst)],True,150,3, cv2.LINE_AA)

            ''''visualize matches'''
            draw_params = dict(matchColor = (0, 255,0), singlePointColor = None, matchesMask = matchesMask, flags = 2)
            img3 = cv2.drawMatches(trainimage, kp1, frame, kp2, good, None, **draw_params)
            plt.imshow(img3, 'gray'), plt.show()
            return dst

        else:
            print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
            matchesMask = None


    def transform_to_grid(self, frame, points):
        zoomed_in = np.float32([[0,0],[0,self.height],[self.width,self.height],[self.width,0]])
        N = cv2.getPerspectiveTransform(points, zoomed_in)
        dst2 = cv2.warpPerspective(frame,N,(self.width,self.height))
        return dst2

    def show_image(self, frame, name='frame'):
        while(True):
            cv2.imshow(name, frame)
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                cv2.destroyAllWindows()
                break
            elif k == ord('s'):
                cv2.imwrite('thisthing.png', frame)

    def segment_frame(self, board):
        gray = cv2.cvtColor(board, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        self.show_image(thresh)

    def detect_corners(self, board):
        gray = cv2.cvtColor(board, cv2.COLOR_BGR2GRAY)
        boardht, boardwd, __ = board.shape
        #ret, gray = cv2.threshold(gray, 127, 255, cv2.THRESH_TRUNC)
        corners = cv2.goodFeaturesToTrack(gray, 25, 0.01, 100)
        corners = np.int0(corners)
        directions = ['nw', 'sw', 'se', 'ne']
        closest_points = dict()

        for i in corners:
            quadrant = ''
            x, y = i.ravel()
            if y<150:
                ry = y
                quadrant = quadrant + 'n'
            elif y > boardht-150:
                ry = boardht-y
                quadrant = quadrant + 's'
            if x<150:
                rx = x
                quadrant = quadrant + 'w'
            elif x > boardwd-150:
                rx = boardht-x
                quadrant = quadrant + 'e'
            if len(quadrant)==2:
                if closest_points.has_key(quadrant):
                    oldsize = closest_points[quadrant][1]
                    newsize = rx*ry
                    if newsize < oldsize:
                        closest_points[quadrant] = [(x, y), rx*ry]
                else:
                    closest_points[quadrant] = [(x, y), rx*ry]
                print(closest_points)

        actual_corners = []
        for point in directions:
            coordinates = closest_points[point][0]
            cv2.circle(board, coordinates, 3, 255, -1)
            actual_corners.append(list(coordinates))

        return np.float32(actual_corners)

    #TODO:
    '''Format to send to the machine learning part'''

if __name__ =='__main__':
    detect = DetectConnectFour(3, 2)
    detect.run()
