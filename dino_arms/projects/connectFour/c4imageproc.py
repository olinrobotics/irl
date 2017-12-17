'''
Purpose: to process a board of Connect4
Authors: Hannah Kolano and Kevin Zhang
Contact: hannah.kolano@students.olin.edu
Last edited: 12/7/17
'''

import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
import img_processing as Process

class DetectConnectFour:
    '''
    Continuously checks the board
    '''

    def __init__(self, numcols, numrows):
        '''initialize the object'''
        self.ready = 0
        self.numcols = numcols
        self.numrows = numrows
        self.width, self.height = numcols*200, numrows*200

        rospy.init_node('boardDetector')
        self.pub_move = rospy.Publisher('opponent_move', Int16, queue_size=10)
        self.sub_ready = rospy.Subscriber('c4_ready', Int16, self.ready_status, queue_size=10)
        self.sub_camera = rospy.Subscriber('usb_cam/image_raw', Image, self.img_callback)

        self.bridge = CvBridge() # converts image types to use with OpenCV

        print("initialized")

    def ready_status(self, data):
        self.ready = data.data

    def update_frame(self):
        ''' DOCSTRING:
            updates frame with the current frame; cuts out Edwin's eyelid
            '''
        self.frame = Process.get_edwin_vision(self.curr_frame)

    def img_callback(self, data):
        ''' DOCSTRING:
        Given img data from usb cam, saves img for later use; called every
        time img recieved from usb cam
        '''
        try:
            # Saves image; converts to opencv format
            self.curr_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print('ERR:%d',e)

    def run(self):
        '''initialize camera'''
        state = 'empty'
        playerColor = 'Nothing'

        while True:
            if self.ready == 1:
                #self.update_frame
                frame = self.curr_frame
                #cap = cv2.VideoCapture(0)
                #_, frame = cap.read()
                #picture = raw_input('Choose a picture')
                frame = cv2.imread(picture)
                frame = cv2.resize(frame,None,fx=.2, fy=.2, interpolation = cv2.INTER_AREA)

                '''find the frames'''
                viewsilhouette = self.extract_black(frame)
                #self.show_image(viewsilhouette, 'silhouetteview')

                '''find the major contour, reduce the field of view'''
                contours = self.draw_contours(viewsilhouette, frame) #, showme = 1)
                x, y, w, h = self.draw_basic_boxes(contours, frame) #, showme = 1)
                board = self.transform_to_grid(frame, np.float32([[x,y],[x,y+h],[x+w,y+h],[x+w,y]]))

                '''warp transform to actual grid'''
                actual_corners = self.detect_corners(board) #, showme = 1)
                board = self.transform_to_grid(board, actual_corners)
                #self.show_image(board, 'after grid')
                current_layout = self.determine_layout(board)
                print(current_layout)

                if state == 'empty':
                    for column in current_layout:
                        if 'Orange' not in column and 'Blue' not in column:
                            pass
                        else:
                            if 'Orange' in column:
                                playerColor = 'Orange'
                            else:
                                playerColor = 'Blue'
                            print('player color is', playerColor)
                            state = 'started'
                            first_ball = True

                if state == 'started':
                    if current_layout != self.layout or first_ball:
                        changed_column, new_color = self.which_column_changed(current_layout, playerColor)
                        changed_column = Int16(changed_column)
                        print('new color is', new_color)
                        if new_color == playerColor:
                            print('changed column', changed_column)
                            self.pub.publish(changed_column)
                        first_ball = False

                self.layout = current_layout
                time.sleep(1)

    def which_column_changed(self, current_layout, playerColor):
        for i in range(len(current_layout)):
            if current_layout[i] != self.layout[i]:
                for j in range(len(current_layout[i])):
                    if current_layout[i][j] != 'Nothing':
                        return i, current_layout[i][j]

    def draw_basic_boxes(self, contours, frame, showme = 0):
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

        if showme !=0:
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
            self.show_image(frame, 'biggest_box')
        return x-10, y-10, w+20, h+20

    def draw_contours(self, thresholdedpic, origpic, showme = 0):
        #TODO:
        ''' check out https://docs.opencv.org/3.3.0/d9/d61/tutorial_py_morphological_ops.html'''
        __, contours, __ = cv2.findContours(thresholdedpic,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(origpic, contours, -1, (0,255,0), 2)
        if showme != 0:
            cv2.drawContours(origpic, contours, -1, (0,255,0), 2)
            self.show_image(origpic, 'contours')
        return contours

    def extract_black(self, frame):
        # convert BGR to HSV
        frame = cv2.blur(frame,(11, 11))
        ret, frame2 = cv2.threshold(frame, 50, 255, cv2.THRESH_BINARY_INV)
        hsv = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        lower_ref = np.array([0, 0, 200])
        upper_ref = np.array([179, 100, 255])

        # Threshold the SSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_ref, upper_ref)
        mask = 255 - mask
        return mask

    def transform_to_grid(self, frame, points):
        zoomed_in = np.float32([[0,0],[0,self.height],[self.width,self.height],[self.width,0]])
        N = cv2.getPerspectiveTransform(points, zoomed_in)
        dst2 = cv2.warpPerspective(frame,N,(self.width,self.height))
        return dst2

    def determine_layout(self, board):
        layout = []
        for i in range(self.numcols):
            layout.append(['']*self.numrows)
        row = 0
        col = 0
        for y in range(self.numcols):
            for x in range(self.numrows):
                this_tile = board[row*200:(row+1)*200, col*200:(col+1)*200]
                color = self.find_ball_color(this_tile) #, showme = 1)
                layout[col][row] = color
                row += 1
            row = 0
            col +=1
        return layout

    def find_ball_color(self, space, showme=0):
        #hsv = cv2.cvtColor(space, cv2.COLOR_BGR2HSV)
        color = 'Nothing'
        lower_orange = np.array([0, 0, 75])
        upper_orange = np.array([50, 50, 250])
        lower_blue = np.array([85, 50, 0])
        upper_blue = np.array([160, 150, 100])

        Omask = cv2.inRange(space, lower_orange, upper_orange)
        Ores = cv2.bitwise_and(space, space, mask=Omask)
        Oresgrey = cv2.cvtColor(Ores, cv2.COLOR_BGR2GRAY)
        __, contoursO, __ = cv2.findContours(Oresgrey,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contoursO:
            if cv2.contourArea(cnt) > 750:
                color = 'Orange'

        Bmask = cv2.inRange(space, lower_blue, upper_blue)
        Bres = cv2.bitwise_and(space, space, mask=Bmask)
        Bresgrey = cv2.cvtColor(Bres, cv2.COLOR_BGR2GRAY)
        __, contoursB, __ = cv2.findContours(Bresgrey,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contoursB:
            if cv2.contourArea(cnt) > 750:
                color = 'Blue'

        if showme != 0:
            self.show_image(Ores, 'Orange')
            self.show_image(Bres, 'Blue')
            print("color found", color)
        return color

    def show_image(self, frame, name='frame'):
        while(True):
            cv2.imshow(name, frame)
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                cv2.destroyAllWindows()
                break
            elif k == ord('s'):
                cv2.imwrite('thisthing.png', frame)

    def detect_corners(self, board, showme = 0):
        gray = cv2.cvtColor(board, cv2.COLOR_BGR2GRAY)
        boardht, boardwd, __ = board.shape
        corners = cv2.goodFeaturesToTrack(gray, 25, .01, 100)
        corners = np.int0(corners)
        directions = ['nw', 'sw', 'se', 'ne']
        closest_points = dict()

        for i in corners:
            quadrant = ''
            x, y = i.ravel()
            cv2.circle(board, (x, y), 3, (0, 0, 255), -1)
            if y<100:
                ry = y
                quadrant = quadrant + 'n'
            elif y > boardht-100:
                ry = boardht-y
                quadrant = quadrant + 's'
            if x<100:
                rx = x
                quadrant = quadrant + 'w'
            elif x > boardwd-100:
                rx = boardht-x
                quadrant = quadrant + 'e'
            if len(quadrant)==2:
                if closest_points.has_key(quadrant):
                    oldsize = closest_points[quadrant][1]
                    newsize = np.fabs(rx*ry)
                    if newsize < oldsize:
                        closest_points[quadrant] = [(x, y), np.fabs(rx*ry)]
                else:
                    closest_points[quadrant] = [(x, y), np.fabs(rx*ry)]
            #print(closest_points)

        actual_corners = []
        for point in directions:
            coordinates = closest_points[point][0]
            cv2.circle(board, coordinates, 7, 255, -1)
            actual_corners.append(list(coordinates))

        if showme != 0:
            self.show_image(board, 'corners')

        return np.float32(actual_corners)

    #TODO:
    '''Format to send to the machine learning part'''

if __name__ =='__main__':
    detect = DetectConnectFour(3, 2)
    detect.run()
