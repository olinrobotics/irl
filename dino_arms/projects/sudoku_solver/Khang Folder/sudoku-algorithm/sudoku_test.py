from sudoku import *

problem_set_1 = [[0, 0, 1], [0, 2, 3], [1, 1, 2], [2, 0, 4], [3, 2, 1]]
problem_set_2 = [[0, 1, 5], [0, 4, 2], [0, 7, 3],
                 [1, 0, 2], [1, 5, 1], [1, 6, 7], [1, 8, 8],
                 [2, 0, 4], [2, 2, 7], [2, 3, 6],
                 [3, 5, 5],
                 [4, 0, 5], [4, 1, 2], [4, 7, 4], [4, 8, 7],
                 [5, 3, 7],
                 [6, 5, 3], [6, 6, 5], [6, 8, 4],
                 [7, 0, 3], [7, 2, 6], [7, 3, 5], [7, 8, 1],
                 [8, 1, 9], [8, 4, 7]]
sudoku = Sudoku(n=2, problem_set=problem_set_1)
# sudoku = Sudoku(n=3, problem_set=problem_set_2)
sudoku.solve()
sudoku.print_sudoku()
