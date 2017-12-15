from Set import *
from Attribute import *
from random import *


class Turn:

    def __init__(self, image_array):
        self.image_array = image_array
        self.card_array = []
        self.coord_array = [(3,0), (3,1), (3,2), (2,0), (2,1), (2,2), (1,0), (1,1), (1,2), (0,0), (0,1), (0,2)]
        #self.coord_array = [(0, 0), (0, 1), (0, 2), (0, 3), (0,4), (1, 0), (1, 1), (1, 2), (1, 3), (1,4)]
        self.create_cards()

    def find_set(self):
        for i1, card1 in enumerate(self.card_array[:-2]):
            for i2, card2 in enumerate(self.card_array[i1 + 1:-1]):
                for card3 in self.card_array[i2 + 1:]:
                    if Set.is_set(card1, card2, card3):
                        return [card1, card2, card3]
        return []

    def create_cards(self):
        for i, c in enumerate(self.image_array):
            str = c[:4]
            #print(str[0], str[1], str[2], str[3])
            color = int(str[0])
            num = int(str[1])
            shape = int(str[2])
            fill = int(str[3])
            card = Set(Color(color), num, Shape(shape), Fill(fill), self.coord_array[i])
            self.card_array.append(card)
        #Turn.print_card_array(self.card_array)


    @staticmethod
    def print_card_array(cards):
        for i, c in enumerate(cards):
            print i+1, ")", c.color.name, c.num, c.shape.name, c.fill.name, c.coord

    @staticmethod
    def create_test():
        card_array = []
        for i in range(1, 4):
            for j in range(1, 4):
                for k in range(1, 4):
                    for l in range(1, 4):
                        card = Set(Color(i), k, Shape(j), Fill(l), 0)
                        # print i, ")", card.shape.name, card.color.name, card.num, card.fill.name
                        card_array.append(card)

        return sample(card_array, 12)


# image = ["1211.jpg", "1312.jpg", "1333.jpg", "2332.jpg", "2211.jpg", "3121.jpg", "1313.jpg", "2221.jpg", "3133.jpg",
#          "1322.jpg", "2331.jpg", "3313.jpg"]
# Turn1 = Turn(image)
# result = Turn1.find_set()
# Turn1.print_card_array(result)
# Turn1 = Turn(Turn.create_test())
# result = Turn1.find_set()
# Turn.print_card_array(result)
