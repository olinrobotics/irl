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
        rospy.init_node('edwin_presence', anonymous = True)

        #subscribing to edwin_bodies, from Kinect
        rospy.Subscriber('body', SceneAnalysis, self.presence_callback, queue_size=10)

        #subscribing to edwin_wave, from Kinect
        rospy.Subscriber('wave_at_me', Int16, self.wave_callback, queue_size=10)

        #subsrcibing to st.py's arm_debug, from Edwin
        rospy.Subscriber('arm_debug', String, self.edwin_location, queue_size=10)

        #setting up ROS publishers to Edwin commands
        self.behavior_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=1)

        # tf transformations between Kinect and Edwin
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        #keeps of the people's coordinates and some statuses about them
        self.peoples = [None]*20

        #coordinates of the person edwin's interacting with
        self.coordx = 0
        self.coordy = 0
        self.coordz = 0

        #edwin's own coordinates
        self.edwinx = 0
        self.edwiny = 0
        self.edwinz = 0

        #keeps track of whether someone waved a Edwin or not
        self.waved = False


    def edwin_location(self, res):
        """
        subscribes to arm_debug to get and parse edwin's current location
        """
        if res.data[0:5] == "WHERE":
            #gets edwin's location
            where = res.data[5:]

            #massive string formatting - takes the string, splits by a formatter, then takes the array index that holds the XYZ,
            #then strips that string and splits it by spacing, and then takes the XYZ
            #reason for this massive formatting is b/c sent data format is not consistent
            where = where.split("\r\n")[2].strip().split('  ')[0:3]

            #makes everything numbers that can be used as coordinates for Edwin
            where = [int(float(coord) * 10) for coord in where]

            self.edwinx = where[0]
            self.edwiny = where[1]
            self.edwinz = where[2]


    def wave_callback(self, waves):
        """
        subscribes to edwin_wave and checks if someone is waving
        """
        if int(waves.data) == 1:
            self.waved = True


    def presence_callback(self, scene):
        """
        subscribes to edwin_bodies and keeps track of people's presence
        """
        #iterates through all 20 possible people that can be tracked
        for index in range(20):
            person = scene.crowd[index]

            #deletes the previous person's presence if they are no longer detected
            if person.ID == 0:
                self.peoples[index] = None
            else:
                xpos, ypos, zpos = self.kinect_transform(person.xpos, person.ypos, person.zpos)

                #either sets up a new presence or updates a person's presence
                if self.peoples[index] is None:
                    self.peoples[index] = Coordinates(person.ID, xpos, ypos, zpos)
                else:
                    self.peoples[index].set_Coordinates(xpos, ypos, zpos)


    def find_new_people(self):
        """
        greets people if they are newly tracked presence,
        responds to waves
        """
        #greets people, only greets once while they're in the camera's view and are center of attention
        for person in self.peoples:
            if (person is not None) and (self.attention() == person.ID) and (person.acknowledged == False):
                print "I see you!", self.attention()

                # greeting = ["R_nudge",
                #             "R_look",
                #             "rotate_hand:: " + str(-1520),
                #             "rotate_wrist:: " + str(-800)]
                # for msg in greeting:
                #     if msg[0] == "R":
                #         self.behavior_pub.publish(msg)
                #     else:
                #         self.arm_pub.publish(msg)
                #     time.sleep(5)

                person.acknowledged = True

        #responds to wave, a completely separate process
        if self.waved == True:
            print "I saw you wave! Hello!"
            msg = "data: R_nudge"
            self.behavior_pub.publish(msg)
            self.waved = False
            time.sleep(3)


    def follow_people(self):
        """
        follows the nearest person's body around
        """
        #finds the person of interest's coordinates and then converts them to Edwin coordinates
        for person in self.peoples:
            if (person is not None) and (self.attention() == person.ID):
                trans = self.kinect_to_edwin_transform(person)
                if trans is not None:
                    xcoord, ycoord, zcoord = self.edwin_transform(trans)
                    print person.ID, xcoord, ycoord, zcoord

                    #the person's coordinates are updated here, edwin's coordinates are updated in the callback
                    self.coordx = xcoord
                    self.coordy = ycoord
                    self.coordz = zcoord

                    #after coordinates are calculated, checks if the person has moved enough to respond, and then responds
                    if abs(self.coordx - self.edwinx) > 400 or abs(self.coordy - self.edwiny) > 400 or abs(self.coordz - self.edwinz) > 400:
                        msg = "move_to:: " + str(self.coordx) + ", " + str(self.coordy) + ", " + str(self.coordz) + ", " + str(11)
                        self.arm_pub.publish(msg)
                        time.sleep(.5)


    def attention(self):
        """
        finds the nearest person and specifically targets them
        """
        center_of_attention = 0
        distance = 5000
        for person in self.peoples:
            if person is not None:
                if person.X < distance: #person's depth is now their X position in edwin frame
                    center_of_attention = person.ID
                    distance = person.X

        if center_of_attention != 0:
            return center_of_attention


    def kinect_to_edwin_transform(self, person):
        """
        transforms coordinates from the kinect reference frame to edwin's reference frame
        kinect reference frame = center of Kinect camera
        edwin reference frame = center of edwin' base, the plateau below his butt
        """
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
        """
        transforms coordinates such that the kinect coordinates are centered
        on the camera rather than to an arbitrary corner
        """
        xposition = x - 320
        yposition = 240 - y
        zposition = z

        return zposition, xposition, yposition


    def edwin_transform(self, coordinates):
        """
        additional transform of coordinates that have been changed from kinect to
        edwin to make sure that edwin can appropriately move to said coordinates
        """
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
        """
        main run function for edwin
        """
        print "running presence detection"
        r = rospy.Rate(10)
        time.sleep(2)
        self.arm_pub.publish("data: set_speed:: 4000")

        while not rospy.is_shutdown():
            self.find_new_people()
            self.follow_people()

            r.sleep()

if __name__ == "__main__":
    detector = Presence()
    detector.run()
