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
import math


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
        if self.variable not in side:
            return str(eval(side))
        if '/' in side or '*' in side:
            indexdiv = side.find('/')
            indexmul = side.find('*')
            if indexmul > indexdiv:
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
            if indexmin > indexplus:
                index = indexmin
                element = '-'
            else:
                index = indexplus
                element = '+'
            left_ele = side[:index]
            right_ele = side[index+1:]
            self.tree = (element, self.tree_base_case_check(left_ele), self.tree_base_case_check(right_ele))
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
        if self.side_w_variable == 'left':
            if type(self.lstree[1]) == str:
                mov_idx = 1
                keep_idx = 2
            elif type(self.lstree[2]) == str:
                mov_idx = 2
                keep_idx = 1

            if self.lstree[0] == '+':
                var_side, non_var_str, var_side_str, non_var_tree = self.plus_operation(self.lstree, self.rsstring, self.lsstring, self.rstree, mov_idx, keep_idx)
            elif self.lstree[0] == '-':
                var_side, non_var_str, var_side_str, non_var_tree = self.minus_operation(self.lstree, self.rsstring, self.lsstring, self.rstree)
            elif self.lstree[0] == '*':
                var_side, non_var_str, var_side_str, non_var_tree = self.multiply_operation(self.lstree, self.rsstring, self.lsstring, self.rstree, mov_idx, keep_idx)
            elif self.lstree[0] == '/':
                var_side, non_var_str, var_side_str, non_var_tree = self.divide_operation(self.lstree, self.rsstring, self.lsstring, self.rstree)

            self.lstree = var_side
            self.rsstring = non_var_str
            self.lsstring = var_side_str
            self.rstree = non_var_tree

        if self.side_w_variable == 'right':
            if type(self.rstree[1]) == str:
                mov_idx = 1
                keep_idx = 2
            elif type(self.rstree[2]) == str:
                mov_idx = 2
                keep_idx = 1

            if self.lstree[0] == '+':
                var_side, non_var_str, var_side_str, non_var_tree = self.plus_operation(self.rstree, self.lsstring, self.rsstring, self.lstree, mov_idx, keep_idx)
            elif self.lstree[0] == '-':
                var_side, non_var_str, var_side_str, non_var_tree = self.minus_operation(self.rstree, self.lsstring, self.rsstring, self.lstree)
            elif self.lstree[0] == '*':
                var_side, non_var_str, var_side_str, non_var_tree = self.multiply_operation(self.rstree, self.lsstring, self.rsstring, self.lstree, mov_idx, keep_idx)
            elif self.lstree[0] == '/':
                var_side, non_var_str, var_side_str, non_var_tree = self.divide_operation(self.rstree, self.lsstring, self.rsstring, self.lstree)

            self.rstree = var_side
            self.lsstring = non_var_str
            self.rsstring = var_side_str
            self.lstree = non_var_tree
        # print(self.lstree)
        # print(self.lsstring)
        # print(self.rstree)
        # print(self.rsstring)

    def plus_operation(self, var_side_tree, non_var_str, var_side_str, non_var_tree, mov_idx, keep_idx):
        non_var_str = eval(str(non_var_str) + '-' + str(var_side_tree[mov_idx]))
        var_side_tree = var_side_tree[keep_idx]
        var_side_str = self.tree_to_string(var_side_tree)
        non_var_tree = self.tree_base_case_check(non_var_tree)
        return var_side_tree, non_var_str, var_side_str, non_var_tree

    def minus_operation(self, var_side_tree, non_var_str, var_side_str, non_var_tree):
        non_var_str = eval(str(non_var_str) + '+' + str(var_side_tree[2]))
        var_side_tree = var_side_tree[1]
        var_side_str = self.tree_to_string(var_side_tree)
        non_var_tree = self.tree_base_case_check(non_var_tree)
        return var_side_tree, non_var_str, var_side_str, non_var_tree

    def multiply_operation(self, var_side_tree, non_var_str, var_side_str, non_var_tree, mov_idx, keep_idx):
        non_var_str = eval(str(non_var_str) + '/' + str(var_side_tree[mov_idx]))
        var_side_tree = var_side_tree[keep_idx]
        var_side_str = self.tree_to_string(var_side_tree)
        non_var_tree = self.tree_base_case_check(non_var_tree)
        return var_side_tree, non_var_str, var_side_str, non_var_tree

    def divide_operation(self, var_side_tree, non_var_str, var_side_str, non_var_tree):
        non_var_str = eval(str(non_var_str) + '*' + str(var_side_tree[2]))
        var_side_tree = var_side_tree[1]
        var_side_str = self.tree_to_string(var_side_tree)
        non_var_tree = self.tree_base_case_check(non_var_tree)
        return var_side_tree, non_var_str, var_side_str, non_var_tree

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
            print(self.lstree)
            print(self.lsstring)
            print(self.rstree)
            print(self.rsstring)
            # prev_answer = answer
            while self.rstree != self.variable and self.lsstring != self.variable:
                self.solve_algebra()
            print(str(self.rsstring) + '=' + str(self.lsstring))
            # if answer != prev_answer:
            #     print(answer)


if __name__ == '__main__':
    ctr = Calculator()
    ctr.eqn = '2*x+4/2-2=4'
    ctr.run()
