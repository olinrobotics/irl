#!/usr/bin/python

import cv2
import numpy as np
import rospy
import struct
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

e_ker = cv2.getStructuringElement(cv2.MORPH_ERODE, (3,3), (1,1))
d_ker = cv2.getStructuringElement(cv2.MORPH_DILATE, (3,3), (1,1))

class OrangeFinder(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.obj_pub = rospy.Publisher('/obj_point', PointStamped, queue_size=10, latch=True)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.im_cb)
        self.cloud_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.cl_cb)
        self.image = None
        self.cloud= None

    def im_cb(self,msg):
        try:
            self.raw_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.GaussianBlur(self.raw_image, (5,5), 0.0)
            cv2.cvtColor(self.raw_image, cv2.COLOR_BGR2HSV, self.raw_image)
            self.image = cv2.inRange(self.raw_image,(4,13,207),(28,255,255))
            cv2.erode(self.image, e_ker, self.image, (1,1), 2)
            cv2.dilate(self.image, d_ker, self.image, (1,1), 3)

        except Exception as e:
            print e

    def cl_cb(self,msg):
        self.cloud = msg
        #self.cloud = pc2.read_points(msg, skip_nans=True, field_names=('x','y','z'))

    def pt_from_cloud(self, cloud, i,j):
        start_idx = cloud.row_step*i + cloud.point_step*j
        x = struct.unpack('f', cloud.data[start_idx:start_idx+4])[0]
        y = struct.unpack('f', cloud.data[start_idx+4:start_idx+8])[0]
        z = struct.unpack('f', cloud.data[start_idx+8:start_idx+12])[0]
        return x,y,z

    def find_orange(self,img):
        ctrs, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        id_ctr = None
        area = 0
        for ctr in ctrs: # find max ctr
            a = cv2.contourArea(ctr)
            if area < a:
                area = a
                id_ctr = ctr
        M = cv2.moments(id_ctr)
        ci = int(M['m01']/M['m00']) 
        cj = int(M['m10']/M['m00'])
        return ci,cj
        
    def publish(self):
        obj_msg = PointStamped()
        obj_msg.header.frame_id = "camera_link"
        obj_msg.header.stamp = rospy.Time.now()
        if self.image != None:
            ci,cj = self.find_orange(self.image.copy())
            #cv2.imshow('wtf', self.image)
            if self.cloud != None:
                pt = self.pt_from_cloud(self.cloud,ci,cj)
                if not np.isnan(pt).any():
                    obj_msg.point.y = -pt[0]
                    obj_msg.point.z = -pt[1]
                    obj_msg.point.x = pt[2]
                    self.obj_pub.publish(obj_msg)

if __name__ == "__main__":
    rospy.init_node('orange_pose')
    o = OrangeFinder()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        o.publish()
        r.sleep()
