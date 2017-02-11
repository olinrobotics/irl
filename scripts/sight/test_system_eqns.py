#!/usr/bin/env python
'''
math_interp.py
Purpose: input a string that is a math equation, output solution
Author: Hannah Kolano
hannah.kolano@studets.olin.edu

HANNAH
MAKE SURE ROSCORE IS RUNNING
RUN IT AS $rosrun edwin math_interp.py
'''
from __future__ import division
import rospy
import rospkg
from std_msgs.msg import String
data = '3*4'


class Calculator:
    def __init__(self):
        '''initializes the object'''
        rospy.init_node('doing_math')
        self.pub = rospy.Publisher('/math_output', String, queue_size=10)
        # rospy.Subscriber('Connors Writing Recog', str, self.cmd_callback)

    def cmd_callback(self, data):
        '''callback'''
        self.equation_to_calc = data

    def removes_equals(self, data):
        '''if there is an = at the end, removes it'''
        if data[-1] == '=':
            data = data[0:-1]
        return data

    def simple_equation(self, eqn):
        '''solves a simple expression'''
        answer = eval(eqn)
        if type(answer) == float:
            answer = "{0:.2f}".format(answer)
        return answer

    def isolate_variable(self, eqn):
        '''gets the variable on one side of the equation'''

    def determine_problem(self, eqn):
        '''determines what type of problem it needs to solve'''
        if type(eval(eqn)) == float or type(eval(eqn)) == int:
            answer = self.simple_equation(eqn)
        else:
            answer = self.isolate_variable(eqn)
        return answer

    def run(self):
        '''
        does the running thing
        '''
        eqn = self.removes_equals(data)
        print(self.determine_problem(eqn))



if __name__ == '__main__':
    ctr = Calculator()
    ctr.run()
