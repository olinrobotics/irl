from enum import Enum


class Color(Enum):
    RED = 0
    GREEN = 1
    BLUE = 2


class Shape(Enum):
    DIAMOND = 0
    WAVY = 1
    CIRCLE = 2


class Fill(Enum):
    SOLID = 0
    DASH = 1
    EMPTY = 2
