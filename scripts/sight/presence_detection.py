#!/usr/bin/env python
import rospy
import math
import numpy as np
from std_msgs.msg import String, Int16
from edwin.msg import *
import time

class Coordinates:
    def __init__(self, x, y, z):
        self.X = x
        self.Y = y
        self.Z = z
        self.acknowledged = false

class Presence:
    def __init__(self):
        rospy.init_node('edwin_presence', anonymous = True)
        rospy.Subscriber('body', String, self.presence_callback, queue_size=10)

        self.behavior_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
        self.arm_pub.publish("data: set_speed:: 3000")

        self.peoples = [None]*20


    def presence_callback(self, bodies):

        info = bodies.data.split(', ')
        personID = int(info[0])
        self.peoples[personID] = Coordinates(int(float(info[1])), int(float(info[2])), int(float(info[3])), false)

        print personID, self.peoples[personID].X,  self.peoples[personID].Y,  self.peoples[personID].Z

        for person in peoples:
            if person is not None and person.ackknowledged is false:
                print "I see you!"
                msg = "data: R_look"
                self.behavior_pub.publish(msg)
                time.sleep(3)
                msg = "data: R_nudge"
                self.behavior_pub.publish(msg)
                time.sleep(3)
                person.ackknowledged = true


    def run(self):
        print "running"
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    detector = Presence()
    time.sleep(2)
    detector.run()
