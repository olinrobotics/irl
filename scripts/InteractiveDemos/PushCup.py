"""Edwin Push-Cup Game
    Connor Novak: connor.novak@students.olin.edu
    Project in human-robot interaction: Edwin pushes cup, human pushes cup

    Overview Position:
    Wrist: 4000 Hand: 2650 Elbow: 8500 Shoulder: 0 Waist: 0
    X: 0.0 Y: 373.5 Z: 407.7 PITCH: 74.5 W(ROLL): 40.5 LEN.: 0.0

    Code Citations:
        [1] http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
        [2] http://docs.opencv.org/trunk/d7/d4d/tutorial_py_thresholding.html
        [3] edwin/scripts/InteractiveDemos TicTacToe.py, lines 663-665
        [4] edwin/scripts/InteractiveDemos TicTacToe.py, line 161
        [5] http://docs.opencv.org/master/d5/d45/tutorial_py_contours_more_functions.html
"""

# Imports
from __future__ import print_function
import roslib
import cv2
import rospy
import sys
import math
import time
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

"""PushCupGame class:
    A class that instantiates and runs a game of Push Cup. Contains state
    variables describing the game and functions to analyze video data and play
    the game.
    | see | | terminal output describing running processes
    | see | | Edwin runs a game of Push Cup
"""
class PushCupGame:

    """__init__ function
        __init__ is a function run once when a new PushCupGame instance is
        created. The function initializes variables to store goal and cup
        positions and other information. It also starts publisher and
        subscriber nodes necessary to run Edwin.
    """
    def __init__(self):

        print ("INIT| Initializing")
        # ---------- State Variables ----------
        # Dimensions
        self.cup_x_prev = 0
        self.cup_y_prev = 0
        self.prev_area = 0
        self.cup_x = 0
        self.cup_y = 0
        self.screen_width = 0
        self.screen_height = 0
        self.gameboard_width = 5000
        self.gameboard_height = 6500

        # Game Elements
        self.timecounter = 0
        self.human_in_frame = False # False if Edwin does not detect human, True if he does
        self.cup_in_frame = False   # False if Edwin does not detect cup, True if he does
        self.goal_in_frame = False  # False if Edwin does not detect goal, True if he does
        self.game_turn = 1          # 0 if human's turn, 1 if Edwin's turn

        rospy.init_node('push_cup') # Creates node from which to subcribe and publish data

        # Sends data to Edwin
        self.behav_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=10)

        # Creates Edwin's overview position route
        time.sleep(2)
        msg1 = "create_route:: R_overview; 0, 3735, 4077, 745, 405, 0"
        print ("INIT| Sending: ", msg1)
        self.arm_pub.publish(msg1)
        time.sleep(2)

        # Moves Edwin along route
        self.overview_pos()
        print ("INIT| Finished")

        # Gets data from usb_cam
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback) # Determines node for subscription

    # Runs once for every reciept of an image from usb_cam
    def callback(self, data):

        # Slows down data processing to once per three frames
        self.timecounter += 1
        if self.timecounter == 3:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converts usb cam feed to csv feed; bgr8 is an image encoding

                # Sets image size
                image_size = cv_image.shape
                screen_height = image_size[0]
                screen_width = image_size[1]
            except CvBridgeError as e:
                print(e)

            self.apply_filter(cv_image)
            self.timecounter = 0
            self.push_cup()

    """ overview_pos function:
        Function: moves Edwin to a standardized position where he can examine the entire gameboard
        -----------------------------------------------------------------------------------
        |param | self | access to the state variables of the class calling the function |
        |see   |      | Edwin moves to position                                          |
    """
    def overview_pos(self):
        msg2 = "run_route:: R_overview"
        print ("INIT| Sending: ", msg2)
        self.arm_pub.publish(msg2)
        time.sleep(3)

    # Manages contours (positions, areas, relevance, etc.)
    def apply_filter(self, feed):

        blur = cv2.GaussianBlur(feed, (5,5), 0) # Gaussian Blur filter

        # Calls functions to contour cup and calculate moments
        contour, contours = self.contour_cup(blur)

        # Returns contoured feed only if 1+ significant contours present in image, else runs raw feed
        if len(contours) > 0:
            video = self.calculate(contour, contours)
            cv2.circle(video,(self.cup_x,self.cup_y),5,(255, 0, 0),-1)
        else:
            video = contour

        # Feed Display(s) for debug:
        #cv2.imshow('Raw Feed (feed)',feed)
        #cv2.imshow('Gaussian Blur Filter (blur)', blur)
        #cv2.imshow('Contour Filter (contour)', contour)

        # Final Contour feed
        cv2.imshow('Final Contours (video)', video)

        k = cv2.waitKey(5) & 0xFF

        return video

    # Contours video feed frame
    def contour_cup(self, video):

         contour = video # Duplicate video feed so as to display both raw footage and final contoured footage

         # Changes BGR video to GRAY and dynamically thresholds it [2]
         vidgray = cv2.cvtColor(video,cv2.COLOR_BGR2GRAY)
         ret,thresh = cv2.threshold(vidgray,100,200,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

         # Uses kernel to clean noise from image (2x) [1]
         kernel = np.ones((5, 5),np.uint8)
         opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)

         # Cleans out background through extra dilations (3x)
         sure_bg = cv2.dilate(opening,kernel,iterations=3)

         # Calculates contours
         contours, h = cv2.findContours(opening,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

         # Creates list of contours with more points than 100 so as to select out for cup and hand
         finalcontours = []
         for cnt in contours:
             area_real = cv2.contourArea(cnt)
             (x,y),radius = cv2.minEnclosingCircle(cnt)
             area_approx = math.pi*radius**2
             area_diff = area_approx - area_real
             #cv2.circle(contour,(int(x),int(y)),int(radius),(0,0,255),2) # Draws circles used for area comparison

             # If contour is really close to circle
             if (area_diff < 1500):
                 finalcontours.append(cnt)
                 self.dotcontour = cnt

         cv2.drawContours(contour, finalcontours, -1, (0,255,0), 3)

         # Feed Display(s) for debug:
         #cv2.imshow('contour_cup: Raw Video(video)',video)
         #cv2.imshow('contour_cup: To GRAY Filter (vidgray)',vidgray)
         #cv2.imshow('contour_cup: Threshold Filter (thresh)',thresh)
         #cv2.imshow('contour_cup: Opening Kernel (opening)', opening)
         #cv2.imshow('contour_cup: Background Clear (sure_bg)', sure_bg)
         #cv2.imshow('contour_cup: Final Video(contour)',contour)

         return contour, finalcontours

    # Center & Movement Detection Function
    def calculate(self, contour, finalcontours):

        # Finds moments and area
        video = contour
        cnt = finalcontours[0] # Defines which contour to apply moments to from list of contours
        moments = cv2.moments(cnt)
        self.area = cv2.contourArea(cnt)

        if len(finalcontours) == 0:
            self.cup_in_frame = False
        else:
            self.cup_in_frame = True

        if moments['m00']!=0:

            # Calculates xy values of centroid
            self.cup_x = int(moments['m10']/moments['m00'])
            self.cup_y = int(moments['m01']/moments['m00'])

        # Checks for human involvement by dividing area values into "cup" and "cup + hand" based on size
        if self.area >= 60000:
            self.human_in_frame = True
        else:
            self.human_in_frame = False

        # Prints values and updates storage variables
        #print ("x = ",self.cup_x,"| y = ",self.cup_y,"| cup_moved = ",self.cup_moved, "| human_in_frame = ",self.human_in_frame, "| area = ", self.area)
        self.cup_x_prev = self.cup_x
        self.cup_y_prev = self.cup_y
        self.area_prev = self.area

        # Feed Display(s) for debug:
        #cv2.imshow('calculate: Raw Video(contour)',contour)
        #cv2.imshow('calculate: Centroid Draw(video)',video)

        return video

    """ push_cup function:
        push_cup makes Edwin push the cup to the goal
        ---------------------------------------------------------------------------------
        |param | self | access to the state variables of the class calling the function |
        |see   |      | Edwin pushes or pulls the cup such that it covers the goal      |
    """
    def push_cup(self):
        pos = self.find_cup()
        print ("PSH| pos = ", pos)
        self.game_turn = 0; # Ends turn

    """ find_cup function
        find_cup converts between the cup's x and y in usb camera feed to x and
        y in realspace (edwin head position)
        ----------------------------------------------------------------------------------
        |param  | self | access to the state variables of the class calling the function |
        |return |      | vector of cup x and y position in realspace                     |
    """
    def find_cup(self):

        # Determines coefficients to convert from camspace to realspace
        x_coefficient = self.screen_width / self.gameboard_width
        y_coefficient = self.screen_height / self.gameboard_height

        # Uses coefficients to convert cup_position to realspace
        edwin_x = x_coefficient * self.cup_x
        edwin_y = y_coefficient * self.cup_y
        return([edwin_x, edwin_y])

    """ play_game function
        play_game holds Edwin's game logic and makes him decide when to move
        and act. The function also calls Edwin's physical moving functions at the
        appropriate times.
        ---------------------------------------------------------------------------------
        |param | self | access to the state variables of the class calling the function |
    """
    def play_game(self):
        if self.cup_in_frame == False and self.goal_in_frame == False:
            if self.game_turn == 1:
                self.push_cup();

    def run(self):
        r = rospy.Rate(20) # Sets update rate
        while not rospy.is_shutdown():
            r.sleep()


if __name__=='__main__':
    pc = PushCupGame() # Creates new instance of class PushCupGame
    pc.run() # Calls run function
