"""finds shapes and draws them on a feed from your webcam"""
import numpy as np
import cv2
import math




while(True):

    img = cv2.imread('square.jpg')  

    imgray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(imgray,127,255,1)
    contours, h = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
     
    for cnt in contours:

        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)

        #checks for a sqaure larger than 10 pixels wide
        if len(approx)==4 and math.fabs(approx.item(0) - approx.item(2)) > 10:

            print "square"   
            cv2.drawContours(img,[cnt],0,(0,0,255),-1)

        """elif len(approx)==5:
            print "pentagon"
            cv2.drawContours(img,[cnt],0,255,-1)

        elif len(approx)==3:
            print "triangle"
            cv2.drawContours(img,[cnt],0,(0,255,0),-1)
        
        elif len(approx)==7:
            print "heptagon"
            cv2.drawContours(img,[cnt],0,(0,0,255),-1)
        elif len(approx) == 9:
            print "half-circle"
            cv2.drawContours(img,[cnt],0,(255,255,0),-1)
        elif len(approx) > 15:
            print "circle"
            cv2.drawContours(img,[cnt],0,(0,255,255),-1)"""

    cv2.imshow('frame',img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()