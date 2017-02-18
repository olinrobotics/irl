#!/usr/bin/env python
'''
math_interp.py
Purpose: input a string that is a math equation, output solution
Author: Hannah Kolano
hannah.kolano@studets.olin.edu

HANNAH
MAKE SURE ROSCORE IS RUNNING
RUN IT AS $rosrun edwin math_interp.py

NEXT STEP:
take out cases of double operatives, etc.
'''
from __future__ import division
import rospy
import rospkg
from std_msgs.msg import String
data = '2*x+1=3'


class Calculator:

    def __init__(self):
        '''initializes the object'''
        rospy.init_node('doing_math')
        self.pub = rospy.Publisher('/math_output', String, queue_size=10)

        self.integer_list = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '.']
        self.order_of_ops = ['-', '+', '/', '*']
        self.operator_list = ['+', '-', '/', '*', '=']
        self.variable_list = ['x', 'y', 'z']
        self.opposite_operation = {'+': '-', '-': '+', '/': '*', '*': '/'}
        self.basics_list = self.integer_list + self.operator_list
        self.basics_and_variables = self.basics_list + self.variable_list

        self.side1 = []
        self.side2 = []
        # rospy.Subscriber('Connors Writing Recog', str, self.cmd_callback)

    def cmd_callback(self, data):
        '''callback'''
        self.equation_to_calc = data

    def removes_equals(self, data):
        '''if there is an = at the end, removes it'''
        if data[-1] == '=':
            data = data[0:-1]
        return data

    def makes_sense(self, data):
        '''if data is something it can solve, send it to simple_equation.
        else, print why it can't solve it.'''
        # self.split_into_list = list(data)
        # for element in self.split_into_list:
        #     if element not in self.basics_list:
        #         if element in self.basics_and_variables:
        #             return self.algebra_solver(data)
        #         else:
        #             print('I do not know how to do that yet :(')
        #             return
        # return self.simple_equation(data)

    def simple_equation(self, eqn):
        '''solves a simple expression'''
        eqn = self.removes_equals(eqn)
        answer = eval(eqn)
        if type(answer) == float:
            answer = "{0:.2f}".format(answer)
        return answer

    def algebra_solver(self, eqn):
        # root_node =
        i = 0
        while i < len(eqn):
            if i in 

    def run(self):
        '''
        does the running thing
        '''
        self.makes_sense(data)

class Node:


if __name__ == '__main__':
    ctr = Calculator()
    ctr.run()
