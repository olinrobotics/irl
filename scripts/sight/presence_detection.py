#!/usr/bin/env python
import rospy
import math
import numpy as np
from std_msgs.msg import String, Int16
from edwin.msg import *
import time
import tf

class Coordinates:
    def __init__(self, ID, x, y, z):
        self.ID = ID
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
        #subscribing to edwin_bodies, from Kinect
        rospy.init_node('edwin_presence', anonymous = True)
        rospy.Subscriber('body', SceneAnalysis, self.presence_callback, queue_size=10)

        #setting up ROS publishers to Edwin commands
        self.behavior_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)
        self.arm_pub.publish("data: set_speed:: 3000")


        # tf transformations
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()



        self.peoples = [None]*20


    def presence_callback(self, scene):

        for index in range(20):
            person = scene.crowd[index]
            if person.ID == 0:
                self.peoples[index] = None
            else:
                # print person.xpos, person.ypos, person.zpos
                xpos, ypos, zpos = self.kinect_transform(person.xpos, person.ypos, person.zpos)
                if self.peoples[index] is None:
                    self.peoples[index] = Coordinates(person.ID, xpos, ypos, zpos)
                else:
                    self.peoples[index].set_Coordinates(xpos, ypos, zpos)


                print self.peoples[index].ID, self.peoples[index].X, self.peoples[index].Y, self.peoples[index].Z

        for person in self.peoples:
            if person is not None and person.acknowledged == False:
                print "I see you!"
                # msg = "data: R_look"
                # self.behavior_pub.publish(msg)
                # time.sleep(2)
                # msg = "data: R_nudge"
                # self.behavior_pub.publish(msg)
                # time.sleep(2)
                person.acknowledged = True


        # print "I am paying attention to person", self.attention()


        for person in self.peoples:
            if person is not None:
                 self.br.sendTransform((person.X, person.Y, person.Z),
                                  tf.transformations.quaternion_from_euler(0, 0, 0),
                                  rospy.Time.now(),
                                  "human",
                                  "kinect")


                 try:
                     (trans,rot) = self.listener.lookupTransform('/world', '/human', rospy.Time(0))
                     print person.ID, trans

                 except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                     pass





    def kinect_transform(self, x, y, z):
        xposition = x - 320
        yposition = 240 - y
        zposition = z

        return zposition, xposition, yposition





    def attention(self):

        center_of_attention = 0
        distance = 5000
        for person in self.peoples:
            if person is not None:
                if person.Z < distance:
                    center_of_attention = person.ID
                    distance = person.Z

        if center_of_attention != 0:
            return center_of_attention


    def run(self):
        print "running"
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    detector = Presence()
    time.sleep(1)
    detector.run()
