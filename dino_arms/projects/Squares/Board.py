import cv2
import numpy as np

class Board:
    def __init__(self, r, c):
        self.rows = r
        self.cols = c
        num_playable_spaces = self.cols * (self.rows + 1) + self.rows * (self.cols + 1)
        self.available_spaces  = dict();
        for i in range(1,num_playable_spaces+1):
            self.available_spaces[i] = True
        self.spaces_in_boxes = []
        self.available_boxes = []
        for i in range(0,self.rows*self.cols):
            first_space = (2 * self.cols + 1) * (i / self.cols) + (i % self.cols) + 1
            box_spaces = [first_space, first_space + self.cols, first_space + self.cols + 1, first_space + 2 * self.cols + 1]
            self.spaces_in_boxes.append(box_spaces)
            self.available_boxes.append(True)
        self.player1_boxes = []
        self.player2_boxes = []

    def make(self):
        space_num = 1;
        for i in range(self.rows):
            line = ""
            for j in range(self.cols):
                line += "+"
                line = self.make_vertical(line, space_num)
                space_num += 1
            line += "+"
            print(line)
            line = ""
            line_up_down = ""
            line_main = ""
            for j in range(self.cols):
                line_main, line_up_down = self.make_horizontal(line_main, line_up_down, space_num)
                line_main += " "
                if not self.available_boxes[3*i+j]:
                    if (3*i+j) in self.player1_boxes:
                        line_main += "X"
                    else:
                        line_main += "O"
                else:
                    line_main += " "
                line_main += "  "
                line_up_down += "    "
                space_num += 1
            line_main, line_up_down = self.make_horizontal(line_main, line_up_down, space_num)
            space_num += 1
            print(line_up_down)
            print(line_main)
            print(line_up_down)
        line = ""
        for j in range(self.cols):
            line += "+"
            line = self.make_vertical(line, space_num)
            space_num += 1
        line += "+"
        print(line)

    def make_horizontal(self, line_main, line_up_down, space_num):
        if self.available_spaces.get(space_num):
            line_main += (str) (space_num)
            if len((str) (space_num)) == 1:
                line_main += " "
            line_up_down += "  "
        else :
            line_main += "| "
            line_up_down += "| "
        return line_main, line_up_down

    def make_vertical(self, line, space_num):
        if self.available_spaces.get(space_num):
            line += "  " + (str) (space_num) + " "
            if len((str) (space_num)) == 1:
                line += " "
        else:
            line += "-----"
        return line

    def fill_line(self, fill_space, player_num):
        self.available_spaces[fill_space] = False
        box_gets_filled = False
        for box in range(self.rows*self.cols):
            if self.box_filled(box) and self.available_boxes[box]:
                self.available_boxes[box] = False
                box_gets_filled = True
                if player_num == 1:
                    self.player1_boxes.append(box)
                else:
                    self.player2_boxes.append(box)
        return box_gets_filled

    def box_filled(self, box_num):
        box = self.spaces_in_boxes[box_num]
        for space in box:
            if self.available_spaces.get(space):
                return False
        return True

    def filled(self):
        return True not in self.available_spaces.values()
