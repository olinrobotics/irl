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
        self.acknowledged = False

    def set_Coordinates(self, x, y, z):
        self.X = x
        self.Y = y
        self.Z = z

    def change_Presence(self):
        self.acknowledged = not self.acknowledged

class Presence:
    def __init__(self):
        rospy.init_node('edwin_presence', anonymous = True)
        rospy.Subscriber('body', SceneAnalysis, self.presence_callback, queue_size=10)

        self.behavior_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
        self.arm_pub.publish("data: set_speed:: 3000")

        self.peoples = [None]*20
        self.history = [None]*20


    def presence_callback(self, scene):

        for index in range(20):
            person = scene.crowd[index]
            if person.ID == 0:
                self.peoples[index] = None
            else:
                if self.peoples[index] is None:
                    self.peoples[index] = Coordinates(person.xpos, person.ypos, person.zpos)
                else:
                    self.peoples[index].set_Coordinates(person.xpos, person.ypos, person.zpos)


                print person.ID, self.peoples[index].X,  self.peoples[index].Y,  self.peoples[index].Z

        for person in self.peoples:
            if person is not None and person.acknowledged == False:
                print "I see you!"
                # msg = "data: R_look"
                # self.behavior_pub.publish(msg)
                time.sleep(3)
                # msg = "data: R_nudge"
                # self.behavior_pub.publish(msg)
                # time.sleep(3)
                person.acknowledged = True


    def run(self):
        print "running"
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    detector = Presence()
    time.sleep(1)
    detector.run()
