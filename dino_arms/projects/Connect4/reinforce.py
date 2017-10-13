#!/usr/bin/env python


import rospy
import numpy as np
import random
import time
from std_msgs.msg import String, Int16

from game_board import C4Board
from rl_brain import QLearningTable


def update():
    for episode in range(100000):
        # initial observation
        observation = env.reset()
        print "EPISODE", episode
        while True:
            # fresh env
            env.render()

            # RL1 choose action based on observation
            action = RL1.choose_action(str(observation))

            # RL take action and get next observation and reward
            observation_, reward1, reward2, done = env.step(action, 1)

            # RL1 learn from this transition
            RL1.learn(str(observation), action, reward1, str(observation_))
            # RL2 learn from this transition
            RL2.learn(str(observation), action, reward2, str(observation_))

            # swap observation
            observation = observation_

            # break while loop when end of this episode
            if done:
                env.render()
                print "GAME ENDED"
                print "VICTOR IS", "P1" if reward1 == 1 else "NO ONE"
                time.sleep(1)
                break

            #show env
            env.render()

            # RL2 choose action based on observation
            action = RL2.choose_action(str(observation))

            # RL take action and get next observation and reward
            observation_, reward1, reward2, done = env.step(action, 2)

            # RL1 learn from this transition
            RL1.learn(str(observation), action, reward1, str(observation_))
            # RL2 learn from this transition
            RL2.learn(str(observation), action, reward2, str(observation_))

            # swap observation
            observation = observation_

            # break while loop when end of this episode
            if done:
                env.render()
                print "GAME ENDED"
                print "VICTOR IS", "P2" if reward2 == 1 else "NO ONE"
                time.sleep(1)
                break


if __name__ == "__main__":
    env = C4Board()
    RL1 = QLearningTable(actions=list(range(env.n_actions)))
    RL2 = QLearningTable(actions=list(range(env.n_actions)))
    update()
