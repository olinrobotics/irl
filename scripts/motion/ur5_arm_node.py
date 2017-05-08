#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('robotic_arm_control')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from math import pi
from namedlist import namedlist

"""
Wrapper for UR5 robotic arm
Direct requests to the UR5 arm should be sent through this node

Before this, the UR driver needs to be brought online

rosrun ur_modern_driver ur5_bringup.launch robot_ip:=10.253.0.51
"""

Route = namedlist('Route', 'joints duration')

def rad2pi(joint_states):
    """control tablet reads in joint positions in degrees
    JointTrajectoryPoint is in radians. we can define as either, but this will convert to radians"""
    return [(j*pi/180) for j in joint_states]

class Arm():
    def __init__(self):
        #Setting up the connection to UR5 server
        rospy.init_node("arm_node", anonymous=True, disable_signals=True)
        self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        self.client.wait_for_server()
        print "Connected to server"

        self.JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(self.JOINT_NAMES):
                self.JOINT_NAMES[i] = prefix + name

        #HOME position of the arm
        self.HOME = [0,-90,0,-90,90,30]

        #Gesture dictionary, and building it
        self.gestures = {}
        self.build_gesture_dict()

        #Setting up subscriber
        self.behav_sub = rospy.Subscriber("behaviors_cmd", String, self.behaviors_callback)
        self.status_pub = rospy.Publisher("arm_status", String, queue_size=0)
        print "Ready to listen"

    def move_to_point(self, point):
        """
        Creates trajectory to a single point
        """

        #TODO: implement
        #implementing this will require looking into UR TCP documentation
        #cannot be done until new head is installed
        pass

    def behaviors_callback(self, data):
        gesture = data.data
        print "GOT GESTURES: ", gesture
        self.run_gesture(gesture)

    def home_robot(self):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.JOINT_NAMES
        try:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position

            #Initialize trajectory with current position of robot
            g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]

            #Add HOME position
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=rad2pi(self.HOME), velocities=[0]*6, time_from_start=rospy.Duration(3)))

            #Send trajectory to arm
            print "Goal created, sending."
            self.client.send_goal(g)
            print "Waiting for result"
            self.client.wait_for_result()
            print "Gesture completed succesfully"

        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

    def build_gesture_dict(self):
        """
        Gesture dictionary for UR arm
        """
        ###SIMON SAYS GESTURES
        self.gestures["dance"] = [Route([83, -128, 40.75, 45,0,0], 2), Route([83, -38, -50, 176.4, 0,0],2)]
        self.gestures["touch_head"] = [Route([-85, -82, -40, -90, 90, 41], 2), Route([-85, -106, -117, -130, 90, 41], 3),
                                    Route([-85, -105, -117, -115, 90, 41], 0.5), Route([-85, -106, -117, -130, 90, 41], 0.5),
                                    Route([-85, -105, -117, -115, 90, 41], 0.5), Route([-85, -106, -117, -130, 90, 41], 0.5),
                                    Route([-85, -82, -40, -90, 90, 41], 2)]
        self.gestures["high5_self"] = [Route([-90, -84, -20.22, -89.94, 90, 52.43],2), Route([-90, -78.18, -118, -128.68, 89.97, 52.4], 4),
                                    Route([-90, -78, -125.85, -153, 89.98, 52.44], 0.5), Route([-90, -78.18, -118, -128.68, 89.97, 52.4], 0.5),
                                    Route([-90, -84, -20.22, -89.94, 90, 52.43],2)]
        self.gestures["dab"] = [Route([0,-65, -82, -43, 90, 41], 2), Route([-64, -96, -96, -42, 125, -38], 1.2),
                                Route([0,-65, -82, -43, 90, 41], 2)]
        self.gestures["starfish"] = [Route([0, -46, -129.13, -49.96, 91.68, 44.96], 3), Route([0,-90,0,-0.33,87.45,44.96], 1), Route(self.HOME, 2)]
        self.gestures["bow"] = [Route(self.HOME, 1.5), Route([0,-91, -98, -40, 88, 42], 3), Route(self.HOME, 2)]
        self.gestures["heart"] = [Route([-7.65,-73.45,-100,-18,91.69,37],2), Route([-49,-85,-77,-22,87,33],1),
                                    Route([-39.5,-96.7,-90,-28,127,42],1), Route([-12, -90, -124, -3, 92, 38.8],1),
                                    Route([38,-75.5,-124,12.36,58,37],1), Route([33.5, -75, -95.5, -15, 54, 26],1),
                                    Route([-7.65,-73.45,-100,-18,91.69,37],1)]
        self.gestures["wave"] = [Route([0,-65, -100, 18, 88, 42], 2),
                                Route([0, -74, -115, -37, 86, 42], 1), Route([0,-65, -100, 18, 88, 42], 1),
                                Route([0, -65, -111, -10, 88, 42], 1)]
        self.gestures["disco"] = [Route([-7.65,-73.45,-100,-18,91.69,37],2), Route([54.63, -104.26, -137.37, 17, 98, 51], 1.5),
                                Route([-1.62, -55.72, -125.98, -4.38, 90.59, 37], 1.5), Route([-63.46, -117, -17, 11, 88, 36], 1.5),
                                Route([-1.62, -55.72, -125.98, -4.38, 90.59, 37], 1.5), Route([54.63, -104.26, -137.37, 17, 98, 51], 1.5),
                                Route([-1.62, -55.72, -125.98, -4.38, 90.59, 37], 1.5), Route([-63.46, -117, -17, 11, 88, 36], 1.5),
                                Route([-1.62, -55.72, -125.98, -4.38, 90.59, 37], 1.5)]
        self.gestures["rub_tummy"] = [Route([-84, -84.9, -30, -81, 89.96, 47.99], 2), Route([-84.5, -75.6, -124, -134.8, 89.96, 47.99], 4),
                                Route([-84, -75.6, -124, -160, 89.96, 47.99], 0.5), Route([-84.5, -75.6, -124, -134.8, 89.96, 47.99], 0.5),
                                Route([-84, -75.6, -124, -160, 89.96, 47.99], 0.5),
                                Route([-84, -84.9, -30, -81, 89.96, 47.99], 2)]
        self.gestures["hug_self"] = [Route([0, -48, -123, -129, 82, 30], 4), Route([-19.3, -47, -122, -129, 82.3, 30], 1), Route([21.2, -47, -122, -111, 82.3, 30], 1),
                                    Route([-19.3, -47, -122, -129, 82.3, 30], 1), Route([21.2, -57, -100, -54, 68.9, 47], 2)]

        ###GENERAL GAME GESTURES
        self.gestures["gloat"] = [Route([1.15, -83, -79.5, -33.88, 91.58, 40.72], 2), Route([1.15, -67, -104, -4.5, 91.5, 40.7], 0.7),
                                Route([1.15, -83, -79.5, -33.88, 91.58, 40.72], 0.7), Route([1.15, -67, -104, -4.5, 91.5, 40.7], 0.7)]
        self.gestures["done_game"] = self.gestures["wave"]
        self.gestures["get_set"] = [Route([0, -31.2, -119, -47, 93.6, 30], 3), Route([-32, -31, -131, -58, 122, 30], 1),
                                    Route([10, -36, -124, -34, 79, 30], 1), Route([-32, -31, -131, -58, 122, 30], 1),
                                    Route([0, -31.2, -119, -47, 93.6, 30], 2)]
        self.gestures["leader"] = [Route([0, -79.9, -87.5, -23.9, 90, 38.9], 2), Route([0, -74.3, -89.7, -128.7, 90, 43], 1),
                                Route([0, -74.3, -89.7, -165.2, 90, 43.1], 0.5), Route([0, -74.3, -89.7, -128.7, 90, 43], 0.5),
                                Route([0, -79.9, -87.5, -23.9, 90, 38.9], 1)]
        self.gestures["look"] = [Route([0, -62, -109, -28, 95, 32], 2)]
        self.gestures["sad"] = [Route([-91, -86.8, -40, -52, 90, 62], 2),
                                Route([-89.2, -78.4, -122.65, -83.4, 90, 62], 3), Route([-89.2, -78.4, -122.65, -113.5, 90, 62], 1),
                                Route([-89.3, -72, -105, -91.2, 90, 62], 3)]
        # self.gestures["praise"] =

        self.gestures["nod"] = [Route([-10.4, -77.81, -99.38, -17.65, 107.77, 37.25], 2), Route([-10.49, -77.81, -99.38, -61, 107.77, 37.25], 1),
                                Route([-10.4, -77.81, -99.38, -17.65, 107.77, 37.25], 1)]

        self.gestures["nod1"] = [Route([4.55, -99.1, -54.45, 23.07, 89.99, 52.43], 2), Route([4.55, -99.1, -54.45, -65, 90, 52.43], 1), Route([4.55, -99.1, -54.45, 23.07, 89.99, 52.43], 1)]
    def run_gesture_incremental(self, gesture):
        """
        Runs a gesture based on what it finds in the dictionary
        Increments, so requires user input to move from keypose to keypose
        """

        gest2run = self.gestures[gesture]
        if type(gest2run[-1]) == int:
            repeat_num = gest2run[-1]
            gest2run.pop(-1)
        else:
            repeat_num = 1
        g = FollowJointTrajectoryGoal()
        try:
            #Add each gesture from the dictionary to the trajectory
            for gest in gest2run:
                if type(gest) == str:
                    cmd = gest.split(":")[0]
                    val = gest.split(":")[1]

                    if cmd == "SLEEP":
                        time.sleep(float(val))
                    continue

                g = FollowJointTrajectoryGoal()
                cur_joint_states = rospy.wait_for_message("joint_states", JointState)
                g.trajectory = JointTrajectory()
                g.trajectory.joint_names = self.JOINT_NAMES

                #Initialize trajectory with current position of robot
                g.trajectory.points = [JointTrajectoryPoint(positions=cur_joint_states.position, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
                g.trajectory.joint_names = self.JOINT_NAMES

                assert len(gest.joints) == 6
                g.trajectory.points.append(JointTrajectoryPoint(positions=rad2pi(gest.joints), velocities=[0]*6, time_from_start=rospy.Duration(gest.duration)))

                #Send trajectory to arm
                print "Goal created, sending."
                self.client.send_goal(g)
                print "Waiting for result"
                self.client.wait_for_result()
                # print "Gesture completed, do next move?"
                inp = raw_input("Ready for next move?")

                if (inp == "n"):
                    raise KeyboardInterrupt

        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise

        except AssertionError:
            print "RUNNING: ", gest.joints
            print "WRONG LENGTH OF JOINTS: GOT %d, EXPECTED 6" % (len(gest.joints))
            raise

    def run_gesture(self, gesture):
        """
        Runs a gesture based on what it finds in the dictionary
        """
        gest2run = self.gestures.get(gesture, None)

        if gest2run is None:
            print "Gesture: ", gesture, " is not recognized"
            return

        self.status_pub.publish("busy")

        if type(gest2run[-1]) == int:
            repeat_num = gest2run[-1]
            gest2run.pop(-1)
        else:
            repeat_num = 1

        g = FollowJointTrajectoryGoal()
        joint_states = rospy.wait_for_message("joint_states", JointState)
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.JOINT_NAMES
        try:
            joints_pos = joint_states.position

            #Initialize trajectory with current position of robot
            g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
            duration = 0

            for repeat in range(repeat_num):
                #Add each gesture from the dictionary to the trajectory
                for gest in gest2run:
                    duration += gest.duration
                    if len(gest.joints) == 6:
                        g.trajectory.points.append(JointTrajectoryPoint(positions=rad2pi(gest.joints), velocities=[0]*6, time_from_start=rospy.Duration(duration)))
                    else:
                        print gest.joints
                        print "WRONG LENGTH: GOT %d, EXPECTED 6" % (len(gest.joints))
                        raise TypeError

            #Send trajectory to arm
            print "Goal created, sending."
            self.client.send_goal(g)
            print "Waiting for result"
            self.client.wait_for_result()
            print "Gesture completed succesfully"

        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

        self.status_pub.publish("free")

    def test_run(self, gesture):
        try:
            inp = raw_input("Ready to run? y/n: ")[0]
            if (inp == 'y'):
                self.run_gesture_incremental(gesture)
            else:
                print "Halting program"
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

    def run_all(self):
        try:
            inp = raw_input("Ready to run? y/n: ")[0]
            if (inp == 'y'):
                self.run_gesture("bow")
                self.run_gesture("starfish")
                self.run_gesture("disco")
                self.run_gesture("heart")
                self.run_gesture("wave")
                self.run_gesture("dab")
                self.run_gesture("nod")
                self.run_gesture("touch_head")
                self.run_gesture("rub_tummy")
                self.run_gesture("clap_left")
                self.run_gesture("gloat")
                self.run_gesture("leader")
                self.run_gesture("sad")
                self.run_gesture("get_set")
                self.run_gesture("hug_self")
                self.home_robot()
                # self.run_gesture("nod1")
            else:
                print "Halting program"
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

    def run(self):
        print "UR5 control is running"
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    a = Arm()
    # a.run()
    a.run_all()
    # a.test_run("hug_self")
    # a.home_robot()
