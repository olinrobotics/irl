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


class Calculator:

    def __init__(self):
        '''initializes the object'''
        rospy.init_node('doing_math')
        # self.pub = rospy.Publisher('/math_output', String, queue_size=10)
        self.eqn = ''
        # rospy.Subscriber('word_publish', String, self.cmd_callback)

        self.integer_list = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '.']
        self.order_of_ops = ['-', '+', '/', '*']
        self.operator_list = ['+', '-', '/', '*', '=']
        self.variable_list = ['x', 'y', 'z']
        self.opposite_operation = {'+': '-', '-': '+', '/': '*', '*': '/'}
        self.basics_list = self.integer_list + self.operator_list
        self.basics_and_variables = self.basics_list + self.variable_list

    def cmd_callback(self, data):
        '''callback'''
        given_string = str(data)
        self.eqn = given_string[6:]

    def removes_equals(self, eqn):
        '''if there is an = at the end, removes it'''
        if eqn[-1] == '=':
            data = eqn[0:-1]
        return data

    def makes_sense(self, eqn):
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
        # eqn = self.removes_equals(eqn)
        answer = eval(eqn)
        if type(answer) == float:
            answer = "{0:.2f}".format(answer)
        return answer

    def initialize_tree(self, eqn):
        if '=' in eqn:
            index = eqn.find('=')
            left_side = eqn[:index]
            right_side = eqn[index+1:]
        right_side = self.tree_base_case_check(right_side)
        left_side = self.tree_base_case_check(left_side)
        print(right_side)
        print(left_side)

    def tree_base_case_check(self, side):
        for element in side:
            if element not in self.integer_list:
                return self.build_tree(side)
        return side

    def build_tree(self, side):
        self.tree = tuple()
        for element in self.order_of_ops:
            if element in side:
                index = side.find(element)
                left_ele = side[:index]
                right_ele = side[index+1:]
                self.tree = (element, self.tree_base_case_check(left_ele), self.tree_base_case_check(right_ele))
        return self.tree

    def check_triviality(self, answer):
        if answer == '':
            return 0
        else:
            return 1

    def run(self):
        '''only prints the answer if it's getting a nontrivial input
        will only print the output once
        '''
        answer = ''
        # while not rospy.is_shutdown():
        if self.check_triviality(self.eqn) == 1:
            prev_answer = answer
            answer = self.initialize_tree(self.eqn)
            if answer != prev_answer:
                print(answer)


if __name__ == '__main__':
    ctr = Calculator()
    ctr.eqn = '2*x+1=3'
    ctr.run()
