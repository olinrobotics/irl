class Cell:
    def __init__(self, n=2, row=0, col=0, potentials=None, number=None):
        self.n = n
        self.row = row
        self.col = col
        self.box = self.get_box()
        if number is not None:
            self.potentials = [number]
        else:
            if potentials is None:
                potentials = range(1, n ** 2 + 1)
            self.potentials = potentials

    def get_number(self):
        """
        Get the value of the cell
        :return: value at cell [row, col] or -1 if there are more than 1 potentials
        """
        if len(self.potentials) == 1:
            return self.potentials[0]
        else:
            return -1

    def set_number(self, number=-1):
        """
        Set num from 1 to n^2. If number is out of range, reset potentials
        :param number: integer
        :return: None
        """

        if number < 1 or number > self.n ** 2:
            self.potentials = range(1, self.n ** 2 + 1)
        else:
            self.potentials = [number]

    def set_potentials(self, potentials=None):
        """
        Set potentials for this cell or init potentials
        :param potentials: an array
        :return: None
        """
        if potentials is None:
            potentials = range(1, self.n ** 2 + 1)
        self.potentials = potentials

    def get_box(self):
        """
        Get the position of box in Sudoku grid based on row and col
        :return: box position
        """
        r = self.row / self.n
        c = self.col / self.n
        return r * self.n + c

    def get_pos_in_box(self):
        """
        Get the position of cell in a box based on row and col
        :return: box position
        """
        r = self.row % self.n
        c = self.col % self.n
        return r * self.n + c

    def get_pos_in_sudoku(self):
        return self.row * self.n ** 2 + self.col