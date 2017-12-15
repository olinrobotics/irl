from Attribute import *


class Set:
    def __init__(self, color, num, shape, fill, coord):
        self.color = color
        self.shape = shape
        self.num = num
        self.fill = fill
        self.coord = coord

    def get_color(self):
        # returns color of object
        print(self.color)

    def matches(self, card):
        # returns 1 if there is any match between two object.
        # returns 0 if there are no matches between the two objects
        if self.color == card.color:
            return 1
        elif self.shape == card.shape:
            return 1
        elif self.num == card.num:
            return 1
        elif self.fill == card.fill:
            return 1
        else:
            return 0

    @staticmethod
    def is_set(card1, card2, card3):
        """
        Check if 3 cards is a set
        :param card1:
        :param card2:
        :param card3:
        :return:
        """
        colors = [card1.color, card2.color, card3.color]
        shapes = [card1.shape, card2.shape, card3.shape]
        fills = [card1.fill, card2.fill, card3.fill]
        nums = [card1.num, card2.num, card3.num]
        if not card1.check_array(colors):
            return False
        if not card1.check_array(shapes):
            return False
        if not card1.check_array(fills):
            return False
        if not card1.check_array(nums):
            return False
        return True

    @staticmethod
    def check_array(array):
        if array[0] == array[1] and array[1] == array[2]:
            return True
        elif array[0] != array[1] and array[1] != array[2] and array[0] != array[2]:
            return True
        else:
            return False

#
# ex1 = Set(Color.BLUE, Shape.CIRCLE, 1, Fill.DASH)
# ex2 = Set(Color.GREEN, Shape.WAVY, 2, Fill.EMPTY)
# ex3 = Set(Color.BLUE, Shape.CIRCLE, 3, Fill.SOLID)

# print ex1.shape
# print Set.is_set(ex1, ex2, ex3)

# print(ex1.matches(ex2))
