#!/usr/bin/env python
import rospy
import math
import numpy as np
from std_msgs.msg import String, Int16
from edwin.msg import *
import time
import tf

class Coordinates:
    """
    helper class to keep track of each individual person's coordinates and status
    """
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
    """
    main class for detecting presence, following people, and waving
    """
    def __init__(self):
        #subscribing to edwin_bodies, from Kinect
        rospy.init_node('edwin_presence', anonymous = True)

        rospy.Subscriber('body', SceneAnalysis, self.presence_callback, queue_size=10)
        rospy.Subscriber('wave_at_me', Int16, self.wave_callback, queue_size=10)


        #setting up ROS publishers to Edwin commands
        self.behavior_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=1)
        self.arm_pub.publish("data: set_speed:: 4000")

        # tf transformations
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        #keeps of the people's coordinates and some statuses about them
        self.peoples = [None]*20

        #coordinates that edwin moves to face the person he's interacting with
        self.coordx = 0
        self.coordy = 0
        self.coordz = 0

        #edwin's own coordinates
        self.edwin = None
        self.edwin = None
        self.edwin = None

        #keeps track of whether someone waved a Edwin or not
        self.waved = False


    def wave_callback(self, waves):

        if int(waves.data) == 1:
            self.waved = True

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

                # print self.peoples[index].ID, self.peoples[index].X, self.peoples[index].Y, self.peoples[index].Z


    def find_new_people(self):

        for person in self.peoples:
            if person is not None and person.acknowledged == False:
                print "I see you!"
                greeting = ["R_nudge",
                            "R_look",
                            "rotate_hand:: " + str(-1520),
                            "rotate_wrist:: " + str(-800)]
                for msg in greeting:
                    if msg[0] == "R":
                        self.behavior_pub.publish(msg)
                    else:
                        self.arm_pub.publish(msg)
                    time.sleep(3)
                person.acknowledged = True

        if self.waved == True:
            print "I saw you wave! Hello!"
            self.behavior_pub.publish(msg)
            msg = "data: R_nudge"
            self.waved = False
            time.sleep(3)

        # print "I am paying attention to person", self.attention()


    def follow_people(self):

        for person in self.peoples:
            if person is not None and self.attention() == person.ID:
                trans = self.kinect_to_edwin_transform(person)
                if trans is not None:
                    xcoord, ycoord, zcoord = self.edwin_transform(trans)
                    print xcoord, ycoord, zcoord

                    if abs(xcoord - self.edwinx) > 400 or abs(ycoord - self.edwiny) > 400 or abs(zcoord - self.edwinz) > 400:
                        self.edwinx = xcoord
                        self.edwiny = ycoord
                        self.edwinz = zcoord
                        msg = "move_to:: " + str(self.edwinx) + ", " + str(self.edwiny) + ", " + str(self.edwinz) + ", " + str(11)
                        self.arm_pub.publish(msg)
                        time.sleep(.5)


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


    def kinect_to_edwin_transform(self, person):

        self.br.sendTransform((person.X, person.Y, person.Z),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            "human",
                            "kinect")

        try:
            (trans,rot) = self.listener.lookupTransform('/world', '/human', rospy.Time(0))
            return trans

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None


    def kinect_transform(self, x, y, z):

        xposition = x - 320
        yposition = 240 - y
        zposition = z

        return zposition, xposition, yposition


    def edwin_transform(self, coordinates):

        edwinx = int(5.485 * coordinates[0] - 1689)
        edwiny = int(7.879 * coordinates[1] - 2794)
        edwinz = int(29.45 * coordinates[2] + 5325)

        if edwinx > 4000:
            edwinx = 4000
        elif edwinx < 0:
            edwinx = 0

        if edwiny > 4000:
            edwiny = 4000
        elif edwiny < -400:
            edwiny = -400

        if edwinz > 3500:
            edwinz = 3500
        elif edwinz < -600:
            edwinz = -600

        return edwinx, edwiny, edwinz


    def run(self):

        print "running"
        r = rospy.Rate(10)
        time.sleep(2)
        while not rospy.is_shutdown():
            self.find_new_people()
            self.follow_people()

            r.sleep()

if __name__ == "__main__":

    detector = Presence()
    detector.run()
