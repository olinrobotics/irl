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
figure out NEGATIVES
figure out how to change 2x to 2*x
'''
from __future__ import division
# import rospy
# import rospkg
# from std_msgs.msg import String
import math


class Calculator:
    def __init__(self):
        '''initializes the object'''
        # rospy.init_node('doing_math')
        # self.pub = rospy.Publisher('/math_output', String, queue_size=10)
        self.eqn = ''
        self.tree = tuple()
        # rospy.Subscriber('word_publish', String, self.cmd_callback)

        self.integer_list = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '.']
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

    def initialize_algebra(self, eqn):
        '''solves algebra'''
        for variable in self.variable_list:
            if variable in eqn:
                self.variable = variable
        self.initialize_tree(eqn)
        self.rsstring = self.tree_to_string(self.rstree)
        self.lsstring = self.tree_to_string(self.lstree)

    def initialize_tree(self, eqn):
        '''takes an equation, splits into two sides. Returns tuples of
        the sides processed into a tree.'''
        index = eqn.find('=')
        left_side = eqn[:index]
        right_side = eqn[index+1:]
        if self.variable in left_side:
            self.side_w_variable = 'left'
        elif self.variable in right_side:
            self.side_w_variable = 'right'
        self.rstree = self.tree_base_case_check(right_side)
        self.lstree = self.tree_base_case_check(left_side)

    def tree_base_case_check(self, side):
        '''takes in one side of the equation. if there's still an
        operation present, keep processsing and return the processed (tree'd)
        side. If not, return itself.'''
        side_string = str(side)
        for element in side_string[1:]:
            if element in self.operator_list:
                return self.build_tree(side)
        return side

    def build_tree(self, side):
        '''take in a side of an equation, create a tree with the
        operations as nodes, returns that tree.'''
        print(side)
        if self.variable not in side:
            return str(eval(side))
        if '/' in side or '*' in side:
            indexdiv = side.rfind('/')
            indexmul = side.rfind('*')
            if indexmul > indexdiv:
                index = indexmul
                element = '*'
            else:
                index = indexdiv
                element = '/'
            left_ele = side[:index]
            right_ele = side[index+1:]
            self.tree = (element, self.tree_base_case_check(left_ele), self.tree_base_case_check(right_ele))

        if ('-' in side and side.find('-') != 0 and side[indexmin-1] not in self.operator_list) or '+' in side:
            indexplus = side.rfind('+')
            indexmin = side.rfind('-')
            # if side[indexmin-1] in self.operator_list:
            #     indexmin = side[:indexmin].rfind('-')
            if indexmin > indexplus:
                index = indexmin
                element = '-'
            else:
                index = indexplus
                element = '+'
            left_ele = side[:index]
            right_ele = side[index+1:]
            self.tree = (element, self.tree_base_case_check(left_ele), self.tree_base_case_check(right_ele))
        print(self.tree)
        return self.tree

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

    def solve_algebra(self):
        '''takes the self.trees and self.strings and does the next appropriate
        operation to them'''
        if self.side_w_variable == 'left':
            vt_vs_nvt_nvs = [self.lstree, self.lsstring, self.rstree, self.rsstring]
        elif self.side_w_variable == 'right':
            vt_vs_nvt_nvs = [self.rstree, self.rsstring, self.lstree, self.lsstring]

        if type(vt_vs_nvt_nvs[0][1]) == str:
            mov_idx = 1
            keep_idx = 2
        elif type(vt_vs_nvt_nvs[0][2]) == str:
            mov_idx = 2
            keep_idx = 1

        if vt_vs_nvt_nvs[0][0] == '+':
            vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3] = self.do_op(vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3], '-', mov_idx, keep_idx)
        elif vt_vs_nvt_nvs[0][0] == '-':
            vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3] = self.do_op(vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3], '+')
        elif vt_vs_nvt_nvs[0][0] == '*':
            vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3] = self.do_op(vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3], '/', mov_idx, keep_idx)
        elif vt_vs_nvt_nvs[0][0] == '/':
            vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3] = self.do_op(vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3], '*')

        vt_vs_nvt_nvs[1] = self.tree_to_string(vt_vs_nvt_nvs[0])
        vt_vs_nvt_nvs[2] = self.tree_base_case_check(vt_vs_nvt_nvs[3])

        if self.side_w_variable == 'left':
            self.lstree, self.lsstring, self.rstree, self.rsstring = vt_vs_nvt_nvs
        elif self.side_w_variable == 'right':
            self.rstree, self.rsstring, self.lstree, self.lsstring = vt_vs_nvt_nvs

        if self.variable in str(self.lsstring):
            self.side_w_variable = 'left'
        elif self.variable in str(self.rsstring):
            self.side_w_variable = 'right'

    def do_op(self, var_side_tree, non_var_str, string, mov_idx=2, keep_idx=1):
        '''given a tree and the opposite side string, snips the operation
        from the tree and moves it to the other side string.'''
        non_var_str = str(non_var_str) + string + str(var_side_tree[mov_idx])
        if self.variable not in non_var_str:
            non_var_str = eval(non_var_str)
        var_side_tree = var_side_tree[keep_idx]
        return var_side_tree, non_var_str

    def parse_negs_and_mul(self, raw_eqn):
        if '-' in raw_eqn:
            index = raw_eqn.find('-')
            if (index == 0 and raw_eqn[index+1] in self.integer_list) or raw_eqn[index-1] in self.operator_list:
                print('I hit this point')
                raw_eqn = raw_eqn[0:index] + '-1*' + raw_eqn[index+1:]
                raw_eqn = self.parse_negs_and_mul(raw_eqn)
        for digit in raw_eqn:
            if digit == self.variable:
                index = raw_eqn.find(digit)
                if type(raw_eqn[index-1]) == int:
                    raw_eqn = raw_eqn[0:index] + '*' + raw_eqn[index:]
                    raw_eqn = self.parse_negs_and_mul(raw_eqn)
        return raw_eqn

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
            self.initialize_algebra(self.eqn)
            # prev_answer = answer
            # print(self.parse_negs_and_mul(self.eqn))
            while self.rstree != self.variable and self.lsstring != self.variable:
                self.solve_algebra()
            print(str(self.rsstring) + '=' + str(self.lsstring))
            # if answer != prev_answer:
            #     print(answer)


if __name__ == '__main__':
    ctr = Calculator()
    ctr.eqn = 'x*-2=2'
    ctr.run()
