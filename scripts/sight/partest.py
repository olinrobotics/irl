class Parse:
    def __init__(self):
        self.equation = '(17+3)/x+((2+1)*3)'
        self.counter = 0
        self.nest_num = 1
        self.placeholders = ['p', 'q', 'r']
        self.place_dict = dict()
        self.par_parsed_eq = self.equation
        while '(' in str(self.par_parsed_eq):
            self.par_parsed_eq = self.parse_par(self.par_parsed_eq)

    def parse_par(self, equation):
        start_par = equation.find('(')

        ind_open = equation[start_par+1:].find('(')
        ind_close = equation[start_par+1:].find(')')

        '''if '''
        if ind_close < ind_open or ind_open == -1:
            self.nest_num += -1
        elif ind_open < ind_close:
            self.nest_num += 1

        if self.nest_num == 0:
            p_holder = self.placeholders[self.counter]
            self.place_dict[p_holder] = equation[start_par+1:ind_close+start_par+1]
            self.counter += 1
            return equation[:start_par] + p_holder + equation[ind_close+start_par+2:]
        else:
            last_close = equation.rfind(')')
            outer_nest = equation[ind_open+start_par+1:last_close]
            inner_nest = self.parse_par(outer_nest)
            self.nest_num += 1
            return equation[:ind_open+start_par+1] + inner_nest + equation[last_close:]


test = Parse()
print(test.par_parsed_eq)