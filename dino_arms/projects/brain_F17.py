#!/usr/bin/env python

import rospy
import time
import numpy as np
from std_msgs.msg import String, Int16

#TODO tons of imports
from Connect4.baseline.connect4 import C4

"""
Kevin Zhang
The Brain for Fall 2017

The integration and accumulation of all the team's projects for this semester. It's
a simple menu interaction that alternates between idle and playing a game.

Note that in order to play some of the games you might need additional dependencies
or things to run for them, refer to those scripts for the games for more info

Simple steps:
1. Goes into idle to start
2. Waits for a game input, after which plays the game
3. Once game is done, can immediately play another game if another person is waiting,
or just go back into idle, at which the loop repeats
"""

class The_Brain(object):

    def __init__(self):
        rospy.init_node("brainf17", anonymous=True)
        self.idle_command = rospy.Publisher("idle_state", String, queue_size=10)
        self.games = {'c':'Connect4', 's':"Set", "t":"Tool Picker", "u":"Sudoku", "b":"Button Game"}
        self.connect = C4(ros_node=rospy)


    def play_game(self, choice):
        """
        giant decision tree to choose a game to play
        """

        if choice == "c":
            # self.connect.run()
            print "PLAYING CONNECT 4"


    def run(self):
        """
        the main interaction sequence, prompts the operator to select what game, and if none,
        then just goes into idle
        """

        print "Brain F17 Module is running"
        running = True
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

            if not running:
                break
            print "GOING INTO IDLE"+"\n"*5
            self.idle_command.publish("go")
            continue_game = True

            while continue_game:
                game = None
                while game not in self.games:
                    game = raw_input("\n\nWhen ready, please choose from the menu: \n"
                        "c: Connect4   s: Set   b: ButtonGame   t: ToolPicker   u: Sudoku\n\n")
                self.idle_command.publish("stop")
                print "PLAYING GAME", self.games[game]
                self.play_game(game)
                print "GAME OVER"+"\n"*5
                c = raw_input("Want to play again now or go into idle? (y or n)\n"
                            "To exit the brain, input 'end'\n\n")
                continue_game = True if c == "y" else False
                running = False if c == "end" else True

        print "\n\nBrain F17 Module turned off"



if __name__=="__main__":
    brain = The_Brain()
    brain.run()
