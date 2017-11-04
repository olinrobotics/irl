import cv2
import numpy as np
from Board import Board

def main():
    rows = input("\nNumber of Rows: ")
    cols = input("Number of Columns: ")
    print("")
    board = Board(rows, cols)
    board.make()
    player_num = 1
    fill_space = input("\nPlayer " + (str) (player_num) +"\nLine to fill: ")
    print("")
    while fill_space != -1:
        box_gets_filled = board.fill_line(fill_space, player_num)
        board.make()
        if board.filled():
            if len(board.player1_boxes) > len(board.player2_boxes):
                print("\nPlayer 1 Wins!\n")
            else:
                print("\nPlayer 2 Wins!\n")
            break;
        if not box_gets_filled:
            if player_num == 1:
                player_num = 2
            else:
                player_num = 1
        fill_space = input("\nPlayer " + (str) (player_num) +"\nLine to fill: ")
        while fill_space != -1 and (fill_space > len(board.available_spaces) or not board.available_spaces.get(fill_space)):
            if fill_space > len(board.available_spaces):
                fill_space = input("\n**Invalid Line** Line to fill: ")
            if not board.available_spaces.get(fill_space):
                fill_space = input("\n**Space Already Filled** Line to fill: ")
        print("")



if __name__ == '__main__':
  main()
