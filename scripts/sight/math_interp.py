#!/usr/bin/env python
'''
math_interp.py
Purpose: input a string that is a math equation, output solution
Author: Hannah Kolano
hannah.kolano@students.olin.edu

NEXT STEPS:
parenthesis (can't deal with more than one set of parenthesis, maybe make self.par a list?)
square root
sin/cos
documentation
'''
from __future__ import division
# import rospy
# import rospkg
# from std_msgs.msg import String
import math
m = math

integer_list = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '.']
operator_list = ['+', '-', '/', '*', '^', '(', ')', '=']
variable_list = ['x', 'y', 'z']
placeholder_list = ['p', 'q', 'r', 's', 't', 'u']


class Calculator:
    def __init__(self):
        '''initializes the object'''
        # rospy.init_node('doing_math')
        # self.pub = rospy.Publisher('/math_output', String, queue_size=10)
        self.eqn = ''
        self.tree = tuple()
        # rospy.Subscriber('word_publish', String, self.cmd_callback)

    def cmd_callback(self, data):
        '''callback'''
        given_string = str(data)
        self.eqn = given_string[6:]

    def solve_simple(self, eqn):
        '''solves a simple expression'''
        eqn = self.removes_equals(eqn)
        if '^' in eqn:
            eqn = eqn[:eqn.find('^')] + '**' + eqn[eqn.find('^')+1:]
        answer = eval(eqn)
        if type(answer) == float:
            answer = "{0:.2f}".format(answer)
        return str(answer)

    def removes_equals(self, eqn):
        '''if there is an = at the end, removes it'''
        if eqn[-1] == '=':
            eqn = eqn[0:-1]
        return eqn

    def initialize_algebra(self, eqn):
        '''solves algebra'''
        for variable in variable_list:
            if variable in eqn:
                self.variable = variable
                break
        eqn = self.parse_var_mul(eqn)
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
        if side_string in placeholder_list:
            return self.build_tree(self.place_dict[side_string])
        elif any((digit in operator_list or digit in placeholder_list)for digit in side_string[1:]):
            return self.build_tree(side)
        return side

    def build_tree(self, side):
        '''take in a side of an equation, create a tree with the
        operations as nodes, returns that tree.'''
        print(side)
        if all(digit in integer_list or digit in operator_list for digit in side):
            return self.solve_simple(side)
        if '(' in side:
            self.place_dict = dict()
            self.counter = 0
            self.tree = self.tree_base_case_check(self.par_parse(side))
            return self.tree

        elif ('-' in side and side.rfind('-') != 0) or '+' in side:
            indexplus = side.rfind('+')
            indexmin = side.rfind('-')
            if indexmin != -1 and side[indexmin-1] in operator_list:
                indexmin = side[:indexmin].rfind('-')
            if indexmin > indexplus:
                index = indexmin
                element = '-'
            elif indexplus > indexmin:
                index = indexplus
                element = '+'
            elif indexplus == -1 and indexmin == -1:
                element = 'NO'
            if element == '+' or element == '-':
                left_ele = side[:index]
                right_ele = side[index+1:]
                self.tree = (element, self.tree_base_case_check(left_ele), self.tree_base_case_check(right_ele))
                return self.tree

        elif '/' in side or '*' in side:
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
            return self.tree

        elif '^' in side:
            indexcarrot = side.rfind('^')
            left_ele = side[:indexcarrot]
            right_ele = side[indexcarrot+1:]
            self.tree = ('^', self.tree_base_case_check(left_ele), self.tree_base_case_check(right_ele))
            return self.tree

    def par_parse(self, equation, nest_num = 1):
        while '(' in str(equation):
            start_par = equation.find('(')

            ind_open = equation[start_par + 1:].find('(')
            ind_close = equation[start_par + 1:].find(')')

            if ind_close < ind_open or ind_open == -1:
                p_holder = placeholder_list[self.counter]
                self.place_dict[p_holder] = equation[start_par + 1:ind_close + start_par + 1]
                self.counter += 1
                equation = equation[:start_par] + p_holder + equation[ind_close + start_par + 2:]
            elif ind_open < ind_close:
                last_close = self.find_corr_close_par(equation[start_par+1:]) + start_par
                outer_nest = equation[ind_open + start_par + 1:last_close+1]
                parsed_outer = self.par_parse(outer_nest, 1)
                equation = equation[:ind_open + start_par + 1] + parsed_outer + equation[last_close+1:]
        return equation

    def find_corr_close_par(self, equation):
        counter = 1
        index = 0
        for digit in equation:
            if digit != '(' and digit != ')':
                index += 1
            else:
                if digit == '(':
                    counter += 1
                elif digit == ')':
                    counter += -1
                if counter == 0:
                    return index
                index += 1

    def tree_to_string(self, tree):
        '''takes a tree and converts it back into a string'''
        equation_string = ''
        if type(tree) == tuple:
            if type(tree[1]) == str and type(tree[2]) == tuple:
                equation_string = tree[1] + tree[0] + self.tree_to_string(tree[2])
            elif type(tree[1]) == tuple and type(tree[2]) == str:
                equation_string = self.tree_to_string(tree[1]) + tree[0] + tree[2]
            elif type(tree[1]) == str and type(tree[2]) == str:
                equation_string = '(' + tree[1] + tree[0] + tree[2] + ')'
            elif type(tree[1]) == tuple and type(tree[2]) == tuple:
                equation_string = self.tree_to_string(tree[1]) + tree[0] + self.tree_to_string(tree[2])
        else:
            equation_string = tree
        return equation_string

    def solve_algebra(self):
        '''takes the self.trees and self.strings and does the next appropriate
        operation to them'''
        if self.side_w_variable == 'left':
            vt_vs_nvt_nvs = [self.lstree, self.lsstring, self.rstree, self.rsstring]
        elif self.side_w_variable == 'right':
            vt_vs_nvt_nvs = [self.rstree, self.rsstring, self.lstree, self.lsstring]

        if self.variable in str(vt_vs_nvt_nvs[0][2]):
            mov_idx = 1
            keep_idx = 2
        elif self.variable in str(vt_vs_nvt_nvs[0][1]):
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
        elif vt_vs_nvt_nvs[0][0] == '^':
            vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3] = self.logs_what(vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3], mov_idx, keep_idx)

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
        non_var_str = str(non_var_str) + string + self.tree_to_string(var_side_tree[mov_idx])
        if self.variable not in non_var_str:
            non_var_str = eval(non_var_str)
        var_side_tree = var_side_tree[keep_idx]
        return var_side_tree, non_var_str

    def logs_what(self, var_side_tree, non_var_str, mov_idx, keep_idx):
        if keep_idx == 1:
            non_var_str = str(eval(str(non_var_str) + '**' + str(eval(('1.0' + '/' + str(var_side_tree[mov_idx]))))))
            var_side_tree = var_side_tree[keep_idx]
            return var_side_tree, non_var_str
        if keep_idx == 2:
            non_var_str = m.log10(eval(non_var_str))/m.log10(eval(var_side_tree[mov_idx]))
            var_side_tree = var_side_tree[keep_idx]
            return var_side_tree, non_var_str

    def parse_var_mul(self, raw_eqn):
        '''if there is a variable with a coefficient, put a multiplication
        sign between them'''
        counter = 0
        for digit in raw_eqn:
            if digit == self.variable:
                index = raw_eqn.find(digit)
                if raw_eqn[index-1] in integer_list and index != 0:
                    raw_eqn = raw_eqn[0:index] + '*' + raw_eqn[index:]
                    raw_eqn = self.parse_var_mul(raw_eqn)
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
            # print(self.tree)
            # prev_answer = answer
            while self.rsstring != self.variable and self.lsstring != self.variable:
                self.solve_algebra()
                if self.rsstring == self.variable:
                    print(str(self.rsstring) + '=' + str(round(self.lsstring, 1)))
                if self.lsstring == self.variable:
                    print(str(self.lsstring) + '=' + str(round(self.rsstring, 1)))
            # if answer != prev_answer:
            #     print(answer)


if __name__ == '__main__':
    ctr = Calculator()
    ctr.eqn = '((2+1)*3)+(17+3)/x=2'
    ctr.run()
