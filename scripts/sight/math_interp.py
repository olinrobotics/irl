#!/usr/bin/env python
'''
math_interp.py
Purpose: input a string that is a math equation, output solution
Author: Hannah Kolano
hannah.kolano@studets.olin.edu
'''
from __future__ import division
import rospy
import rospkg
from std_msgs.msg import String
data = '6/4='


class Calculator:
    def __init__(self):
        rospy.init_node('doing_math')
        self.pub = rospy.Publisher('/math_output', String, queue_size=10)
        # rospy.Subscriber('Connors Writing Recog', str, self.cmd_callback)
        rospack = rospkg.RosPack()

    def cmd_callback(self, data):
        self.equation_to_calc = data

    def removes_equals(self, data):
        if data[-1] == '=':
            data = data[0:-1]
        return data

    def run(self):
        '''
        does the running thing
        '''
        eqn = self.removes_equals(data)
        answer = eval(eqn)
        print(answer)


if __name__ == '__main__':
    ctr = Calculator()
    ctr.run()
