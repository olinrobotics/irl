from cell import *
import copy


class Sudoku:
    def __init__(self, n=2, problem_set=None):
        self.n = n
        self.cells = []
        self.rows = []
        self.cols = []
        self.boxes = []
        self.uncompleted = []
        self.problem_set = problem_set
        self.init_sudoku_from(problem_set)
        self.solution = []

    def init_sudoku_from(self, problem_set):
        """
        Set up sudoku from problem_set
        :param problem_set: array of (row, col, number)
        :return: None
        """
        self.basic_init()

        for element in problem_set:
            r = element[0]
            c = element[1]
            num = element[2]
            cell = Cell(n=self.n, row=r, col=c, number=num)
            self.cells[cell.get_pos_in_sudoku()].set_number(cell.get_number())

        self.uncompleted = [cell for cell in self.uncompleted if cell.get_number() == -1]

    def basic_init(self):
        """
        Init cells, rows, cols, boxes, uncompleted arrays
        :return: None
        """
        self.rows = self.init_2d_array(self.n ** 2)
        self.cols = self.init_2d_array(self.n ** 2)
        self.boxes = self.init_2d_array(self.n ** 2)
        self.uncompleted = [0 for _ in range(self.n ** 4)]

        for r in range(self.n ** 2):
            for c in range(self.n ** 2):
                cell = Cell(n=self.n, row=r, col=c)
                self.cells.append(cell)
                self.uncompleted[cell.get_pos_in_sudoku()] = cell
                self.rows[r][c] = cell
                self.cols[c][r] = cell
                self.boxes[cell.get_box()][cell.get_pos_in_box()] = cell

    def update_uncompleted(self, cell):
        """
        Remove cell from uncompleted and add it to solution
        :return: None
        """
        self.uncompleted.remove(cell)
        self.solution.append(cell)

    def solve(self):
        """
        Solve the sudoku and store the solution in solution[]
        Only find the first possible solution
        :return: None
        """
        # Flag to exit the loop if we can't find a solution
        flag = len(self.uncompleted)
        while True:
            self.update_cells_potentials()
            self.check_unique_in_potentials()

            # If len still stays the same, need to pick a random number from potentials
            if flag == len(self.uncompleted):
                break
            else:
                flag = len(self.uncompleted)

        # If flag > 0, we need to make some guesses
        if flag > 0:
            # Make some guesses by picking a cell with the least number of potentials
            self.sort_uncompleted()
            cell = self.uncompleted[0]

            for potential in cell.potentials:
                # Create other states of sudoku with possible potentials and try to solve it
                child_sudoku = copy.deepcopy(self)
                child_cell = child_sudoku.rows[cell.row][cell.col]
                child_cell.set_number(potential)
                child_sudoku.update_uncompleted(child_cell)
                child_sudoku.solve()

                # If a solution found in this child sudoku, return the solution to its parent(s)
                if child_sudoku.check_solution():
                    remove_cells = []
                    for uncompleted_cell in self.uncompleted:
                        uncompleted_cell.set_number(
                            child_sudoku.cells[uncompleted_cell.get_pos_in_sudoku()].get_number())
                        remove_cells.append(uncompleted_cell)

                    for c in remove_cells:
                        self.update_uncompleted(c)

                    break

    def update_cells_potentials(self):
        """
        Run through uncompleted array and update cells' potentials.
        If potentials has only one number, remove cell from uncompleted.
        Run until we can't reduce num of potentials in each uncompleted cell.
        :return: None
        """
        # if run_again > 0, then run this func again
        run_again = 0
        remove_cells = []
        for cell in self.uncompleted:
            self.update_potentials(cell)
            if cell.get_number() != -1:
                remove_cells.append(cell)
                run_again += 1

        for cell in remove_cells:
            self.update_uncompleted(cell)

        if run_again > 0:
            self.update_cells_potentials()

    def update_potentials(self, cell):
        """
        Update potentials of a specific cell in sudoku
        :param cell: a cell in sudoku
        :return: None
        """
        if cell.get_number() != -1:
            return

        # Array of integer number with size = n
        existing_nums = [0 for _ in range(self.n ** 2)]

        # Check row cells
        for row_cell in self.rows[cell.row]:
            if row_cell.get_number() != -1:
                existing_nums[row_cell.get_number() - 1] += 1

        # Check col cells
        for col_cell in self.cols[cell.col]:
            if col_cell.get_number() != -1:
                existing_nums[col_cell.get_number() - 1] += 1

        # Check box cells
        for box_cell in self.boxes[cell.box]:
            if box_cell.get_number() != -1:
                existing_nums[box_cell.get_number() - 1] += 1

        # Update potentials of the cell
        for index, value in enumerate(existing_nums):
            if value > 0:
                if index + 1 in cell.potentials:
                    cell.potentials.remove(index + 1)

    def sort_uncompleted(self):
        """
        Sort uncompleted array based on the number of potentials
        :return: None
        """
        self.uncompleted.sort(key=lambda cell: len(cell.potentials), reverse=False)

    def check_unique_in_potentials(self):
        """
        Check potentials of uncompleted array to find a unique potential
        :return: None
        """
        remove_cell = []
        for cell in self.uncompleted:
            for potential in cell.potentials:
                row_cells = self.rows[cell.row]
                col_cells = self.cols[cell.col]
                box_cells = self.boxes[cell.box]
                if self.is_unique_in(row_cells, potential) or self.is_unique_in(col_cells, potential) \
                        or self.is_unique_in(box_cells, potential):
                    cell.set_number(potential)
                    remove_cell.append(cell)
                    break

        for cell in remove_cell:
            self.update_uncompleted(cell)

    @staticmethod
    def is_unique_in(cells, potential):
        """
        Checks if a potential is unique in its row, column or box
        :param cells: an array of cells in row, col or box
        :param potential: int
        :return: True if a potential is unique
        """
        count = 0
        for cell in cells:
            if potential in cell.potentials:
                count += 1
                if count > 1:
                    return False
        return True

    def check_solution(self):
        """
        Check if a sudoku's solution is correct or a sudoku is solvable with such a solution
        :return:
        """
        return len(self.uncompleted) == 0

    @staticmethod
    def init_2d_array(n):
        """
        Helper method inits empty a 2D array with size = n * n
        :param n:
        :return: 2D array size n * n
        """
        array = []
        for _ in range(n):
            temp_array = [0 for _ in range(n)]
            array.append(temp_array)
        return array

    def print_sudoku(self):
        """
        Helper method prints the Sudoku for visualization
        :return: None
        """
        for i, row in enumerate(self.rows):
            if i % self.n == 0:
                for _ in range(self.n ** 2 + self.n + 1):
                    print "-",
                print ""
            for j, cell in enumerate(row):
                if j % self.n == 0:
                    print "|",
                if cell.get_number() == -1:
                    print "0",
                else:
                    print cell.get_number(),
            print "|"

        for _ in range(self.n ** 2 + self.n + 1):
            print "-",
