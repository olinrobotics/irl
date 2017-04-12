#!/usr/bin/env python
import rospy
import cv2
import cv2.cv as cv
import numpy as np
from scipy import stats

from std_msgs.msg import String
from sensor_msgs.msg import Image
from edwin.msg import Edwin_Shape
from cv_bridge import CvBridge, CvBridgeError

class visualmenu:
    #The goal is to have someone put their hand under the camera, and then
    #the camera will read which portion of the screen the hand is in.
    #If the person keeps their hand relatively still for 3 seconds,
    #determined by increasing the contour area by 50%, then the
    #Dimensions of a camera screen are 640 x 480.

    def __init__(self):
        #self.choice_pub = rospy.Publisher("choice_cmd", String, queue_size=10)

        #self.game_state = rospy.Subscriber("all_control", String, self.cmd_callback)


        #self.bridge = CvBridge()
        #rospy.Subscriber("usb_cam/image_raw", Image, self.img_callback)
        #self.image_sub = rospy.subscriber("usb_cam/camera_raw", Image, self.image_callback)

        #the areas.

        self.range1 = [[0,210], [0,480]] #minx, maxx, miny, maxy
        self.range2 = [[211,429], [0,480]]
        self.range3 = [[430,640], [0,480]]

        self.list1 = self.get_area_coords(self.range1)
        self.list2 = self.get_area_coords(self.range2)
        self.list3 = self.get_area_coords(self.range3)

        self.area_list = [self.list1, self.list2, self.list3]

        self.detect = True
        self.cap = cv2.VideoCapture(0)
        self.median = []


    def img_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            h, w = self.frame.shape[:2]
        except CvBridgeError as e:
            print(e)

    def cmd_callback(self, data):
        print "GOT: ", data.data
        if "vm stop" in data.data:
            self.detect = False
        elif "vm go" in data.data:
            self.started_tracking = time.time()
            self.detect = True

    def findHand(self, frame):

        composite = frame[0]
        blur = frame[1]
        #Find the hand
        max_area = 0

        contours, hierarchy = cv2.findContours(composite.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for i in range(len(contours)):
            cont=contours[i]
            area = cv2.contourArea(cont) #Theoretically the largest contour is a hand.
            if(area>max_area):
                max_area=area
                ci=i
        cont=contours[ci]

        #find center coordinate of the hand, or rather, the centroids
        m = cv2.moments(cont)
        cx = int(m['m10']/m['m00'])
        cy = int(m['m01']/m['m00'])

        coords = (cx,cy)

        #cv2.circle(blur, coords, 10, (255, 0, 0), 2)
        cv2.imshow("composite", composite)
        #cv2.imshow("blur", blur)
        cv2.waitKey(1)
        return coords

    def process_frame(self, frame, cList):
        #Image pre-processing
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blur = cv2.GaussianBlur(frame,(5,5),0)
        #ret,thresh1 = cv2.threshold(blur,10,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        frame_list = [];

        for i in cList:
            blur_copy = blur.copy()
            colorHigh = i + 10
            colorLow = i - 15

            mask = cv2.inRange(blur_copy, colorLow, colorHigh)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)



            frame_list.append(mask)


        composite = sum(frame_list)
        composite = cv2.medianBlur(composite, 5)

        frames = [composite, blur]
        return frames


    def check_pos(self, coords):
        xy = coords
        #print xy

        for i in self.area_list:
            if xy in i:

                return self.area_list.index(i) #Is there a simpler way?


    def get_area_coords(self, params):
        coord_list = []
        for x in range(params[0][0], params[0][1]):
            for y in range(params[1][0], params[1][1]):
                coord_list.append((x,y))
        return coord_list

    def calibrate_color(self):
        #With an initial guess, the person calibrates their hand color onto the machine,
        #the loop will auto-adjust until it reaches an "optimal" value, and decides the
        #range from there.
        cList = self.initialize_reference()
        roi_params = (160, 40, 480, 440)#x,y, length, height
        total_pixels = 640*480 - 480*440

        # while True:
        #     k = cv2.waitKey(100) & 0xFF
        #     if k == ord('q'):
        #         cv2.destroyAllWindows()
        #         print cList
        #         return cList
        #     else:
        #         #The auto-adjust functions by reading how much "white" is detected
        #         # in the non-ROI.  It will try to minimize the outside while maximizing
        #         #the inside.
        #         frame = self.get_frame()
        #
        #         composite = self.process_frame(frame, cList)
        #         composite = composite[0]
        #
        #         roi = composite.copy()
        #         roi = roi[roi_params[0]:roi_params[0]+roi_params[2],
        #             roi_params[1]:roi_params[1]+roi_params[3]]
        #
        #         total_black = 640*480 - cv2.countNonZero(composite)
        #         roi_black = 480*440 - cv2.countNonZero(roi)
        #
        #         bg_noise = total_pixels - (total_black - roi_black)
        #         cv2.imshow("composite", composite)
        #         print bg_noise
        return cList

                #http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
                #http://simena86.github.io/blog/2013/08/12/hand-tracking-and-recognition-with-opencv/
                #http://www.pyimagesearch.com/2016/02/15/determining-object-color-with-opencv/

    def initialize_reference(self):
        colorGuess = np.array([115,160, 229]) #skin tone guess
        #229,160, 115

        p1 = (325,135)
        p2 = (325,305)
        p3 = (245, 305)
        p4 = (405, 305)
        p5 = (245, 250)
        p6 = (405, 250)

        pList = [p1, p2, p3, p4, p5, p6]


        cList = [[],[],[],[],[],[]]



        while True:
            k = cv2.waitKey(100) & 0xFF
            if k == ord('q'):
                break
            else:
                frame = self.get_frame()

                cv2.putText(frame,
                "Put your hand in the box and press 'q' when you are ready to calibrate",
                (40, 460), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 2)

                #Overall frame for the hand
                cv2.rectangle(frame, (160, 40), (480, 440), colorGuess, 3)

                #Visual marker for each detection point.
                for i in pList:
                    point1 = tuple((np.asarray(i) - 5))
                    point2 = tuple((np.asarray(i) + 5))
                    cv2.rectangle(frame, point1, point2, colorGuess, -1)

                # for i in range(0, len(pList)):
                #     point = pList[i]
                #     #retrieve each RGB value separately, compile into tuple
                #     cList[i] = (int(frame[point][0]), int(frame[point][1]), int(frame[point][2]))



                #process each point
                # cList = np.array(cList)
                # colorGuess = np.mean(cList, axis=0)

                cv2.imshow("test", frame)


            #Once target is set, find the median
        for j in range(0,50):
            frame = self.get_frame()
            #Overall frame for the hand
            cv2.rectangle(frame, (160, 40), (480, 440), colorGuess, 3)

            #Visual marker for each detection point.
            for i in pList:
                point1 = tuple((np.asarray(i) - 5))
                point2 = tuple((np.asarray(i) + 5))
                cv2.rectangle(frame, point1, point2, colorGuess, -1)

            for k in range(0,len(cList)):
                point = pList[k]
                cList[k].append((int(frame[point][0]), int(frame[point][1]), int(frame[point][2])))


            cv2.imshow("test", frame)
            cv2.waitKey(1)

        for l in range(0,len(cList)): #pare down to finding the medium
            cList[l] = np.mean(cList[l], axis=0)

        cv2.destroyAllWindows()
        return cList

    def get_frame(self):
        grab, frame = self.cap.read()
        frame = cv2.flip(frame, 1)
        return frame

    def get_median(self, option): #Technically the mode, but that would be confusing.
        if len(self.median) < 50:
            self.median.append(option)
        elif len(self.median) == 50:
            self.median.pop(0)
            self.median.append(option)
            med = self.median[:]
            med = stats.mode(med)
            med = int(med[0])

            return med




    def dispMenu(self, frame, coords,option):



        if option == 0:
            cv2.rectangle(frame, (0,0), (210, 480), (255,0,0), -1)
        elif option == 1:
            cv2.rectangle(frame, (211,0), (429, 480), (0,255,0), -1)
        elif option == 2:
            cv2.rectangle(frame, (430,0), (640, 480), (0,0,255), -1)

        cv2.circle(frame, coords, 10, (255, 0, 0), 2)
        cv2.imshow("Menu", frame)
        cv2.waitKey(1)


    def run(self):
        #r = rospy.Rate(10)
        #time.sleep(2)

        #while not rospy.is_shutdown():
        colorList = self.calibrate_color()

        while self.detect:
            frame = self.get_frame()
            processed_frame = self.process_frame(frame, colorList)
            coords = self.findHand(processed_frame)
            option = self.check_pos(coords)

            self.dispMenu(frame, coords, option)



            # if option == 0:
            #     pass #run this node.
            # elif option == 1:
            #     pass #run the other node.
            # #r.sleep()

if __name__ == '__main__':
    vm = visualmenu()
    vm.run()
