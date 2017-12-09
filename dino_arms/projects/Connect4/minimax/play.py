# Python Final Project
# Connect Four
#
# Erik Ackermann
# Charlene Wang
#
# Play connect four
# February 27, 2012

from connect4 import *

def main():
    u""" Play a game!
    """

    g = Game()
    g.printState()
    player1 = g.players[0]
    player2 = g.players[1]

    win_counts = [0, 0, 0] # [p1 wins, p2 wins, ties]

    exit = False
    while not exit:
        while not g.finished:
            g.nextMove()

        g.findFours()
        g.printState()

        if g.winner == None:
            win_counts[2] += 1

        elif g.winner == player1:
            win_counts[0] += 1

        elif g.winner == player2:
            win_counts[1] += 1

        printStats(player1, player2, win_counts)

        while True:
            play_again = unicode(raw_input(u"Would you like to play again? "))

            if play_again.lower() == u'y' or play_again.lower() == u'yes':
                g.newGame()
                g.printState()
                break
            elif play_again.lower() == u'n' or play_again.lower() == u'no':
                print u"Thanks for playing!"
                exit = True
                break
            else:
                print u"I don't understand... ",

def printStats(player1, player2, win_counts):
    print u"{0}: {1} wins, {2}: {3} wins, {4} ties".format(player1.name,
        win_counts[0], player2.name, win_counts[1], win_counts[2])

if __name__ == u"__main__": # Default "main method" idiom.
    main()
