#!/usr/bin/python

#######################TO DO:###############################
# Backlogged 1&2: --> 1a. DONE publishing center bottom pixel of can and doing ROS stuff
# 		1. need to take calibration image using alphabot camera
# 		2. pre calculate camera focal length and hardcode it into processing to save time
# 3. DONEish integrate with ROS
# 4. clean up this mess :)
# ## Figure out how to only publish frames with contours if listeners, dont do now bc info size
#


# import the necessary packages
import numpy as np

#ROS communication
import rospy
import tf
import roslib

from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point, PointStamped

pt_pub = None
last_published = None

def to_msg(d): # convert detection to PointStamped message
    res = PointStamped()
    res.header.frame_id = 'camera_link'
    res.header.stamp = d.pose.header.stamp
    p = d.pose.pose.position
    res.point.x = p.z
    res.point.y = -p.x
    res.point.z = -p.y
    return res

def tag_cb(msg):
    global last_published
    for d in msg.detections:
        p_msg = to_msg(d)
        if d.id == 0:
            now = rospy.Time.now()
            if( (now - last_published).to_sec() > 1.0):
                last_published = now
                pt_pub.publish(p_msg) # discovery point

def main():
    global pt_pub, last_published
    #initialize ROS channels
    rospy.init_node('april_tag_finder')
    last_published = rospy.Time.now()
    img_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_cb)
    pt_pub = rospy.Publisher('/obj_point', PointStamped, queue_size=1) # pretend to be can
    rospy.spin()

if __name__ == "__main__":
    main()
