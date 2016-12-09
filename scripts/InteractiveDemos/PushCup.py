"""Edwin Push-Cup Game
    Connor Novak: connor.novak@students.olin.edu
    Project in human-robot interaction: Edwin pushes cup, human pushes cup

    Overview Position:
    Wrist: 3550 Hand: 2900 Elbow: 7000 Shoulder: 3500 Waist: 800
    X: 62.1 Y: 483.2 Z: 373.9 PITCH: 84.5 W(ROLL): 19.5 LEN.: 0.0

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
        # Positions
        self.cup_pos = [0,0]
        self.goal_pos = [0,0]

        # Dimensions
        self.screen_width = 640
        self.screen_height = 480
        self.gameboard_top = 5000
        self.gameboard_bottom = 6500
        self.gameboard_left = 0
        self.gameboard_right = 0

        # Motion Limits
        self.lowlimit_x = -1500
        self.highlimit_x = 2300
        self.lowlimit_y = 3200
        self.highlimit_y = 6700
        self.lowlimit_z = -1000
        self.highlimit_z = 4000

        # Game Elements
        self.timecounter = 0
        self.human_in_frame = False # False if Edwin does not detect human, True if he does
        self.cup_in_frame = False   # False if Edwin does not detect cup, True if he does
        self.goal_in_frame = False  # False if Edwin does not detect goal, True if he does
        self.game_turn = 0          # 0 if human's turn, 1 if Edwin's turn

        rospy.init_node('push_cup') # Creates node from which to subcribe and publish data

        # Sends data to Edwin
        self.behav_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.arm_pub = rospy.Publisher('/arm_cmd', String, queue_size=10)

        # Creates Edwin's overview position route
        time.sleep(2)
        msg1 = "create_route:: R_overview; 621, 4832, 3739, 845, 195, 0" # Positions multiplied by 10 from #s at top of code
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
        contour, contours = self.contour_feed(blur)

        # Returns contoured feed only if 1+ contours present in image, else runs raw feed
        if len(contours) > 0:
            video = self.calculate(contour, contours)

            # Draws tracking dots
            cv2.circle(video,(self.cup_pos[0],self.cup_pos[1]),5,(255, 0, 0),-1)
            cv2.circle(video,(self.goal_pos[0],self.goal_pos[1]),5,(0,255,0),-1)
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
    def contour_feed(self, video):

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
        finalcontours = [None]*3 # 1st Elem: Cup | 2nd Elem: Goal | 3rd Elem: Hand
        for cnt in contours:
            area_real = cv2.contourArea(cnt)
            print(area_real)
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            area_approx = math.pi*radius**2
            area_diff = area_approx - area_real
            diff_coefficient = area_diff / radius
            #cv2.circle(contour,(int(x),int(y)),int(radius),(0,0,255),2) # Draws circles used for area comparison

            # If contour is really close to circle
            if (diff_coefficient < 100):
                if (area_real < 200):
                    finalcontours[1] = cnt
                elif (area_real > 200):
                    finalcontours[0] = cnt
            if (area_real > 20000):
                    finalcontours[2] = cnt



        cv2.drawContours(contour, finalcontours, -1, (0,255,0), 3)

        # Feed Display(s) for debug:
        #cv2.imshow('contour_feed: Raw Video(video)',video)
        #cv2.imshow('contour_feed: To GRAY Filter (vidgray)',vidgray)
        #cv2.imshow('contour_feed: Threshold Filter (thresh)',thresh)
        #cv2.imshow('contour_feed: Opening Kernel (opening)', opening)
        #cv2.imshow('contour_feed: Background Clear (sure_bg)', sure_bg)
        #cv2.imshow('contour_feed: Final Video(contour)',contour)

        return contour, finalcontours

    # Center & Movement Detection Function
    def calculate(self, contour, finalcontours):

        video = contour

        #--------------------Unpacks finalcontours--------------------#

        # Cup contour
        if (finalcontours[0] != None):
            cup_contour = finalcontours[0]
            self.cup_in_frame = True
            cup_moments = cv2.moments(cup_contour)

            # Calculates xy values of centroid
            if cup_moments['m00']!=0:
                self.cup_pos[0] = int(cup_moments['m10']/cup_moments['m00'])
                self.cup_pos[1] = int(cup_moments['m01']/cup_moments['m00'])
        else:
            self.cup_in_frame = False

        # Goal contour
        if (finalcontours[1] != None):
            goal_contour = finalcontours[1]
            self.goal_in_frame = True
            goal_moments = cv2.moments(goal_contour)

            # Calculates xy values of centroid
            if goal_moments['m00']!=0:
                self.goal_pos[0] = int(goal_moments['m10']/goal_moments['m00'])
                self.goal_pos[1] = int(goal_moments['m01']/goal_moments['m00'])
        else:
            self.goal_in_frame = False

        # Hand contour
        if (finalcontours[2] != None):
            hand_contour = finalcontours[2]
            self.hand_in_frame = True
        else:
            self.hand_in_frame = False

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
        pos = self.convert_space(self.cup_pos[0],self.cup_pos[1])
        print ("PSH| pos = ", pos)
        self.game_turn = 0; # Ends turn

    """ convert_space function
        convert_space converts an xy point in camspace (pixel location) to
        realspace (edwin head location)
        ------------------------------------------------------------------------
        |param  | self | access to the state variables of the class calling    |
        |the function                                                          |
        |param  | x    | x position of point in camspace                       |
        |param  | y    | y position of point in camspace                       |
        |return |      | vector of cup x and y position in realspace           |
    """
    def convert_space(self, x, y):

        # Determines coefficients to convert from camspace to realspace
        x_coefficient = self.gameboard_width / self.screen_width
        y_coefficient = self.gameboard_height / self.screen_height

        # Uses coefficients to convert input position to realspace coordinates
        x_real = x_coefficient * x
        y_real = y_coefficient * y
        return([x_real, y_real])

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
