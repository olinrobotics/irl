from Set import *
from Attribute import *
from random import sample


class Game:
    def __init__(self, card_array):
        """
        Initialize a game with a card array
        :param card_array: a card array of any length
        """
        self.card_array = card_array

    def find_set(self):
        """
        Find if there is a set in card array
        :return: a set of 3 cards or an empty array
        """
        for i1, card1 in enumerate(self.card_array[:-2]):
            for i2, card2 in enumerate(self.card_array[i1 + 1:-1]):
                for card3 in self.card_array[i2 + 1:]:
                    if Set.is_set(card1, card2, card3):
                        return [card1, card2, card3]
        return []

    @staticmethod
    def print_card_array(cards):
        """
        Helper method helps print out a card array
        :param cards: a card array of any length
        """
        for i, c in enumerate(cards):
            print i, ")", c.shape.name, c.color.name, c.num, c.fill.name

    @staticmethod
    def create_test():
        """
        Helper method creates a test of 12 random cards
        :return: 12 random different cards
        """
        # card_array contains all possible cards
        card_array = []
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    for l in range(3):
                        card = Set(Color(i), Shape(j), k, Fill(l))
                        card_array.append(card)

        # Take 12 random cards from the card_array
        return sample(card_array, 12)


# game1 = Game(Game.create_test())
# result = game1.find_set()
# Game.print_card_array(result)
