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
figure out NEGATIVES and figure out how to change 2x to 2*x
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
        self.tree = tuple()
        # rospy.Subscriber('word_publish', String, self.cmd_callback)

        self.integer_list = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '.']
        self.order_of_ops = [('*', '/'), ('+', '-')]
        self.operator_list = ['+', '-', '/', '*', '=']
        self.variable_list = ['x', 'y', 'z']
        self.opposite_operation = {'+': '-', '-': '+', '/': '*', '*': '/'}
        self.basics_list = self.integer_list + self.operator_list
        self.basics_and_variables = self.basics_list + self.variable_list

    def cmd_callback(self, data):
        '''callback'''
        given_string = str(data)
        self.eqn = given_string[6:]

    def solve_simple(self, eqn):
        '''solves a simple expression'''
        eqn = self.removes_equals(eqn)
        answer = eval(eqn)
        if type(answer) == float:
            answer = "{0:.2f}".format(answer)
        return answer

    def removes_equals(self, eqn):
        '''if there is an = at the end, removes it'''
        if eqn[-1] == '=':
            data = eqn[0:-1]
        return data

    def solve_algebra(self, eqn):
        '''solves algebra'''
        sides = self.initialize_tree(eqn)
        self.rightsidetree = sides[0]
        self.leftsidetree = sides[1]
        self.rightsidestring = self.tree_to_string(self.rightsidetree)
        self.leftsidestring = self.tree_to_string(self.leftsidetree)

    def initialize_tree(self, eqn):
        '''takes an equation, splits into two sides. Returns tuples of
        the sides processed into a tree.'''
        if '=' in eqn:
            index = eqn.find('=')
            left_side = eqn[:index]
            right_side = eqn[index+1:]
        right_side = self.tree_base_case_check(right_side)
        left_side = self.tree_base_case_check(left_side)
        return right_side, left_side

    def tree_base_case_check(self, side):
        '''takes in one side of the equation. if there's still an
        operation present, keep processsing and return the processed
        side. If not, return itself.'''
        for element in side:
            if element in self.operator_list:
                return self.build_tree(side)
        return side

    def build_tree(self, side):
        '''take in a side of an equation, create a tree with the
        operations as nodes, returns that tree.'''
        # print(side)
        if '/' in side or '*' in side:
            indexdiv = side.find('/')
            indexmul = side.find('*')
            if indexdiv == -1:
                index = indexmul
                element = '*'
            elif indexmul == -1:
                index = indexdiv
                element = '/'
            else:
                if indexmul < indexdiv:
                    index = indexmul
                    element = '*'
                else:
                    index = indexdiv
                    element = '/'
            left_ele = side[:index]
            right_ele = side[index+1:]
            self.tree = (element, self.tree_base_case_check(left_ele), self.tree_base_case_check(right_ele))

        if '-' in side or '+' in side:
            indexplus = side.find('+')
            indexmin = side.find('-')
            if indexplus == -1:
                index = indexmin
                element = '-'
            elif indexmin == -1:
                index = indexplus
                element = '+'
            else:
                if indexmin < indexplus:
                    index = indexmin
                    element = '-'
                else:
                    index = indexplus
                    element = '+'
            left_ele = side[:index]
            right_ele = side[index+1:]
            self.tree = (element, self.tree_base_case_check(left_ele), self.tree_base_case_check(right_ele))
        return self.tree

    def unpack_tree(self, rightside, leftside):
        '''takes a tree tuple and changes it back to a string'''
        pass

    def tree_to_string(self, tree):
        '''takes a tree and converts it back into a string'''
        equation_string = ''
        if type(tree) == tuple:
            if type(tree[1]) == str and type(tree[2]) == tuple:
                equation_string = tree[1] + tree[0] + self.tree_to_string(tree[2])
            elif type(tree[1]) == tuple and type(tree[2]) == str:
                equation_string = self.tree_to_string(tree[1]) + tree[0] + tree[2]
            elif type(tree[1]) == str and type(tree[2]) == str:
                equation_string = tree[1] + tree[0] + tree[2]
            elif type(tree[1]) == tuple and type(tree[2]) == tuple:
                equation_string = self.tree_to_string(tree[1]) + tree[0] + self.tree_to_string(tree[2])
        elif type(tree) == str:
            equation_string = tree
        return equation_string

    def check_triviality(self, answer):
        '''returns 1 if getting a value from the subscriber;
        otherwise returns 0'''
        if answer == '':
            return 0
        else:
            return 1

    def run(self):
        '''only prints the answer if it's getting a nontrivial input
        will only print the output once '''
        answer = ''
        # while not rospy.is_shutdown():
        if self.check_triviality(self.eqn) == 1:
            prev_answer = answer
            self.solve_algebra(self.eqn)
            print(self.leftsidetree)
            print(self.rightsidetree)
            # print(left_answer)
            # if answer != prev_answer:
            #     print(answer)


class Equation(Calculator):
    def __init__(self):
        self.left_tuple = tuple()
        self.right_tuple = tuple()
        self.left_string = ''
        self.right_string = ''

    def __str__(self):
        return '%d = %d' % (self.left_string, self.right_string)

    def string_to_tree(self, side):
        side_tree = self.tree_base_case_check(side)
        return side_tree


if __name__ == '__main__':
    ctr = Calculator()
    ctr.eqn = '5*x/2+3=5'
    ctr.run()
