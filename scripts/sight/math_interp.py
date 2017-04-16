#!/usr/bin/env python
'''
math_interp.py
Purpose: input a string that is a math equation, output solution
Author: Hannah Kolano
hannah.kolano@students.olin.edu

NEXT STEPS:
error checking
documentation
'''
from __future__ import division
import rospy
import rospkg
from std_msgs.msg import String
import math
m = math

integer_list = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '.']
operator_list = ['+', '-', '/', '*', '^', '(', ')', '=']
variable_list = ['x', 'y', 'z']
placeholder_list = ['p', 'q', 'r', 's', 't', 'u']


class Calculator:
    def __init__(self):
        '''initializes the object'''
        rospy.init_node('doing_math')
        self.pub = rospy.Publisher('/math_output', String, queue_size=10)
        self.eqn = ''
        self.tree = tuple()
        rospy.Subscriber('word_publish', String, self.cmd_callback)

    def cmd_callback(self, data):
        '''callback'''
        given_string = str(data)
        self.eqn = given_string[6:]

    def solve_simple(self, eqn):
        '''solves a simple expression'''
        if eqn[-1] == '=':
            eqn = eqn[0:-1]
        if '^' in eqn:
            eqn = eqn[:eqn.find('^')] + '**' + eqn[eqn.find('^')+1:]
        answer = eval(eqn)
        if type(answer) == float:
            answer = "{0:.2f}".format(answer)
        return str(answer)

    def initialize_algebra(self, eqn):
        '''solves algebra'''
        if not all(digit in variable_list or digit in integer_list or digit in operator_list for digit in eqn):
            raise ValueError('I found something weird')
        found_variables = []
        for variable in variable_list:
            if variable in eqn:
                self.variable = variable
                index = self.eqn.find(variable)
                found_variables.append(variable)
                if self.eqn[index+1].find(variable) != -1:
                    raise ValueError('I found two instances of the same variable')
        if len(found_variables) > 1:
            raise ValueError('I found too many variables')
        else:
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

        # If there are parenthesis, go through them first
        if '(' in side:
            self.place_dict = dict()
            self.counter = 0
            self.tree = self.tree_base_case_check(self.par_parse(side))
            return self.tree

        # check first for addition and subtraction
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

        # then check for division or multiplication
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

        # then check for powers
        elif '^' in side:
            indexcarrot = side.rfind('^')
            left_ele = side[:indexcarrot]
            right_ele = side[indexcarrot+1:]
            self.tree = ('^', self.tree_base_case_check(left_ele), self.tree_base_case_check(right_ele))
            return self.tree

    def par_parse(self, equation):
        '''takes an equation and returns a parsed version of it.
        also adds placeholders and what they represent into self.place_dict'''
        while '(' in str(equation):
            start_par = equation.find('(')

            ind_open = equation[start_par + 1:].find('(')
            ind_close = equation[start_par + 1:].find(')')

            # if it closes before a new one opens, add it to self.place_dict
            if ind_close < ind_open or ind_open == -1:
                p_holder = placeholder_list[self.counter]
                inside_par = equation[start_par + 1:ind_close + start_par + 1]
                if all(digit in operator_list or digit in integer_list for digit in inside_par):
                    p_holder = self.solve_simple(inside_par)
                else:
                    self.place_dict[p_holder] = inside_par
                    self.counter += 1
                equation = equation[:start_par] + p_holder + equation[ind_close + start_par + 2:]
            # if another one opens, recursively go inside until it gets to one that closes
            elif ind_open < ind_close:
                last_close = self.find_corr_close_par(equation[start_par+1:]) + start_par
                outer_nest = equation[ind_open + start_par + 1:last_close+1]
                parsed_outer = self.par_parse(outer_nest)
                equation = equation[:ind_open + start_par + 1] + parsed_outer + equation[last_close+1:]
        return equation

    def find_corr_close_par(self, equation):
        '''given an equation starting after a beginning (, returns the index of the corresponding )'''
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
        # vt - variable side tree; vs - variable side string; nvt - nonvariable side tree; nvs - nonvariable side string
        if self.side_w_variable == 'left':
            vt_vs_nvt_nvs = [self.lstree, self.lsstring, self.rstree, self.rsstring]
        elif self.side_w_variable == 'right':
            vt_vs_nvt_nvs = [self.rstree, self.rsstring, self.lstree, self.lsstring]

        # figures out where in the tree the variable is
        if self.variable in str(vt_vs_nvt_nvs[0][2]):
            mov_idx, keep_idx = 1, 2
        elif self.variable in str(vt_vs_nvt_nvs[0][1]):
            mov_idx, keep_idx = 2, 1

        # input the right variables and signs and do the operation
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

        # reassign to the attributes
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
        '''is the do_op operation but with powers.'''
        if keep_idx == 1:
            non_var_str = str(eval(str(non_var_str) + '**' + str(eval(('1.0' + '/' + str(var_side_tree[mov_idx]))))))
            var_side_tree = var_side_tree[keep_idx]
            return var_side_tree, non_var_str
        if keep_idx == 2:
            non_var_str = m.log10(eval(str(non_var_str)))/m.log10(eval(str(var_side_tree[mov_idx])))
            var_side_tree = var_side_tree[keep_idx]
            return var_side_tree, non_var_str

    def parse_var_mul(self, raw_eqn):
        '''if there is a variable with a coefficient, put a multiplication
        sign between them'''
        for digit in raw_eqn:
            if digit == self.variable or digit == '(':
                if digit == self.variable:
                    index = raw_eqn.find(digit)
                elif digit == '(':
                    index = raw_eqn.find(digit)
                if raw_eqn[index-1] in integer_list and index !=0:
                    raw_eqn = raw_eqn[0:index] + '*' + raw_eqn[index:]
                    raw_eqn = self.parse_var_mul(raw_eqn)
        return raw_eqn

    def determine_problem(self):
        '''figures out what type of problem it is'''
        if any(digit in variable_list for digit in self.eqn):
            try:
                self.initialize_algebra(self.eqn)
                while self.lstree != self.lsstring or self.rstree != self.rsstring:
                    self.solve_algebra()
                if self.rsstring == self.variable:
                    return self.lsstring
                elif self.lsstring == self.variable:
                    self.rsstring
            except ValueError as err:
                print(err)
        elif all(digit in variable_list or digit in integer_list or digit in operator_list for digit in self.eqn):
            return self.solve_simple(self.eqn)
        else:
            print('what?')

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
        while not rospy.is_shutdown():
            if self.check_triviality(self.eqn) == 1:
                prev_answer = answer
                self.determine_problem()
                if answer != prev_answer:
                    print(answer)



if __name__ == '__main__':
    ctr = Calculator()
    # ctr.eqn = '3x=@'
    ctr.run()
