#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String

class LoadedPath(object):

    def __init__(self, filepath):
        data = pandas.read_csv(filepath)
        self.hand_path = zip(data['hand_x'],data['hand_y'],data['hand_z'])
        self.elbow_path = zip(data['elbow_x'],data['elbow_y'],data['elbow_z'])
        self.times = list(data['times'].values)

class OriginalPlanner(object):

    def __init__(self, original_folder):
        rospy.init_node('original_funky_moves')
        self.plan_publisher = rospy.Publisher("plan", Path, queue_size=10)
        self.request_subscriber = rospy.Subscriber("plan_request", PlanRequest, self.make_plan)
        self.original_folder = original_folder

    def make_plan(self, request):
        # load points
        print "loading path"
        path_name = request.dmp_name
        paths = LoadedPath(self.original_folder+"/"+path_name+".csv") #not other OS compatible, but neither is ROS, so whatevs

        #convert to path
        path = self.makePath(paths.hand_path)

        # publish plan
        print "publishing"
        self.plan_publisher.publish(path)

    def makePath(self,plan):
        path = [ArmPos(x=p[0], y=p[1], z=p[2]) for p in plan]
        return Path(path=path)

    def execute(self):
        rospy.spin()