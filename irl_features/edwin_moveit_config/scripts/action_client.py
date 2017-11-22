#!/usr/bin/python
#edwin_controller/follow_joint_trajectory

import numpy as np
import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

N_JOINTS = 5

class EdwinClient:
    def __init__(self):
        #arm_name should be b_arm or f_arm
        self.jta = actionlib.SimpleActionClient('/edwin/edwin_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

    def move_joint(self, angles):
        goal = FollowJointTrajectoryGoal()                  
        goal.trajectory.joint_names = ['joint_{}'.format(i+1) for i in range(N_JOINTS)]
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(3) 
        goal.trajectory.points.append(point)
        print self.jta.send_goal_and_wait(goal)

def main():
    cli = EdwinClient()
    print 'initialization complete'
    cnt = 3.14
    while not rospy.is_shutdown():
        vals = np.zeros(5)
        #vals = np.random.random_sample(5) * 6.28
        #print vals
        cli.move_joint(list(vals))


if __name__ == '__main__':
    rospy.init_node('joint_test_node')
    main()
