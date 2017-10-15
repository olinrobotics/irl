from Attribute import *


class Set:
    def __init__(self, color=Color(0), shape=Shape(0), num=0, fill=Fill(0)):
        self.color = color
        self.shape = shape
        self.num = num
        self.fill = fill

    @staticmethod
    def is_set(card1, card2, card3):
        """
        Determine if 3 cards is a set
        :param card1:
        :param card2:
        :param card3:
        :return: true if 3 cards make a set
        """
        colors = [card1.color, card2.color, card3.color]
        shapes = [card1.shape, card2.shape, card3.shape]
        fills = [card1.fill, card2.fill, card3.fill]
        nums = [card1.num, card2.num, card3.num]
        attributes = [colors, shapes, fills, nums]
        for attr in attributes:
            if not Set.check_array(attr):
                return False
        return True

    @staticmethod
    def check_array(attr):
        """
        Determine if attributes are all the same or all different
        :param attr: attributes of 3 cards
        :return: true if all attributes are all the same or all different
        """
        assert attr is not None and len(attr) == 3

        if attr[0] == attr[1] and attr[1] == attr[2]:
            return True
        elif attr[0] != attr[1] and attr[1] != attr[2] and attr[0] != attr[2]:
            return True
        else:
            return False

    @staticmethod
    def test_func():
        """
        Generate 3 cards to test functions
        :return: true if 3 cards below make a set
        """
        # ex1 = Set(Color.BLUE, Shape.CIRCLE, 1, Fill.DASH)
        ex1 = Set()
        ex2 = Set(Color.GREEN, Shape.WAVY, 2, Fill.EMPTY)
        ex3 = Set(Color.BLUE, Shape.CIRCLE, 3, Fill.SOLID)
        print Set.is_set(ex1, ex2, ex3)


# Set.test_func()
