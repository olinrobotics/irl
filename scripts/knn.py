#!/usr/bin/env python
import rospkg
import rospy
import math
import time

import csv
from std_msgs.msg import String
from edwin.msg import Bones
from sklearn.neighbors import NearestNeighbors
import pandas as pd
import random

#with open('file.csv', newline='') as csvfile:
#     spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
#     for row in spamreader:
#         print(', '.join(row))
#def loadDataset(filename, split, trainingSet=[] , testSet=[]):
trainingSet= []
testSet = []
with open('skeleton.csv', 'r') as csvfile:
    lines = csv.reader(csvfile)
    print(lines)
    dataset= list(lines)
    for x in range(len(dataset)-1):
        #for y in range(4):
        #    dataset[x][y] = float(dataset[x][y])
        if random.random() < .8 :
            trainingSet.append(dataset[x])
        else:
            testSet.append(dataset[x])
