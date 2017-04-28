#!/usr/bin/env python
import rospkg
import rospy
import math
import time

import csv
from std_msgs.msg import String
from edwin.msg import Bones
from sklearn.neighbors import KNeighborsClassifier
import pandas as pd
import random

#with open('file.csv', newline='') as csvfile:
#     spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
#     for row in spamreader:
#         print(', '.join(row))
#def loadDataset(filename, split, trainingSet=[] , testSet=[]):
"""def E_distance(data1, data2, len_data):
    distance = 0
    for x in data1:
        if x > 1:
            distance += (data1[x]-data2[x])**2
    return math.sqrt(distance)
"""



"""with open('skeleton.csv', 'rU') as file:
    label= [col['A'] for col in csv.reader(file)]
    training_data = [col['E':'AE'] for col in csv.reader(file)
list_of_files = ['l_hand_head','l_hand_r_hand','l_hand_r_shoulder','l_hand_stomach',
                 'r_hand_head','r_hand_l_shoulder','r_hand_stomach']

df = pd.read_csv(skeleton.csv)
training_data= df['E':'AE']
print(df['E':'AE'])
"""


"""list_of_files = ['l_hand_head','l_hand_r_hand','l_hand_r_shoulder','l_hand_stomach',
                 'r_hand_head','r_hand_l_shoulder','r_hand_stomach']
for i in list_of_files:
    #with open('skeleton.csv','rU') as file:
    with open(i, 'rU') as file:
    #table = [row for row in csv.reader(file)]
    for row in csv.reader(file):
        write= csv.writer(file)
        writer =writerow(row)"""


class Gestures:
    def __init__(self,skeleton):
        self.skeleton = skeleton
        rospy.init_node('gestures', anonymous=True)
        rospy.Subscriber('/skeleton', Bones, self.data_callback)
        self.number =0
        self.bow = 0
        self.disco = 0
        self.check = 0
        self.pub = rospy.Publisher('skeleton_points', String, queue_size=10)
        self.trainingSet = []
        self.testSet = []
        self.labels_test =[]
        self.labels_train = []
        self.data_from_sub = []
        self.gestures = {"bow":0,"dab":0,"disco":0,"heart":0,"high_five":0,"hug":0,
          "rub_tummy":0,"star":0,"touch_head":0,"wave":0}
        with open('skeleton.csv', 'r') as csvfile:
            lines = csv.reader(csvfile)
            labels =[]
            dataset =[]
            for rows in lines:
                labels.append(rows[0])
                dataset.append(rows[1:])
            dataset= dataset[1:]
            #labels= labels[1:]
            #print(dataset)
            for i in dataset:
                for l in range(len(i)):
                    if l % 3 == 0:
                        i[l]= float(i[l])+float(i[3])
                    if l % 3 == 1:
                        i[l]= float(i[l])+float(i[4])
                    if l % 3 == 2:
                        i[l]= float(i[l])+float(i[5])
            #print(labels)
            #print(dataset)
            for x in range(len(dataset)-1):
                if random.random() < .8 :
                    self.trainingSet.append(dataset[x])
                    self.labels_train.append(labels[x])
                else:
                    self.testSet.append(dataset[x])
                    self.labels_test.append(labels[x])
        self.neighbors= KNeighborsClassifier()
        time.sleep(2)
        #recieving data in forms of a list of tuples

    def data_callback(self,data):
        self.data_from_sub = []
        self.data_from_sub.extend([data.h.x,data.h.y,data.h.z])
        self.data_from_sub.extend([data.n.x,data.n.y,data.n.z])
        self.data_from_sub.extend([data.t.x,data.t.y,data.t.z])
        self.data_from_sub.extend([data.rs.x,data.rs.y,data.rs.z])
        self.data_from_sub.extend([data.re.x,data.re.y,data.re.z])
        self.data_from_sub.extend([data.rh.x,data.rh.y,data.rh.z])
        self.data_from_sub.extend([data.ls.x,data.ls.y,data.ls.z])
        self.data_from_sub.extend([data.le.x,data.le.y,data.le.z])
        self.data_from_sub.extend([data.lh.x,data.lh.y,data.lh.z])
        for l in range(len(self.data_from_sub)):
            if l % 3 == 0:
                self.data_from_sub[l]= float(self.data_from_sub[l])+float(self.data_from_sub[3])
            if l % 3 == 1:
                self.data_from_sub[l]= float(self.data_from_sub[l])+float(self.data_from_sub[4])
            if l % 3 == 2:
                self.data_from_sub[l]= float(self.data_from_sub[l])+float(self.data_from_sub[5])

    def training(self):
        self.neighbors = KNeighborsClassifier(n_neighbors=7)
        self.neighbors.fit( self.trainingSet, self.labels_train)

    def machine_learning(self):
        #print(self.data_from_sub)
        if self.disco == 1:
            if self.neighbors.predict([self.data_from_sub])[0] == 'disco2':
                self.pub.publish('disco')
            elif self.neighbors.predict([self.data_from_sub])[0] == 'disco1':
                self.pub.publish('wave')
            else:
                self.disco = 0
        elif self.neighbors.predict([self.data_from_sub])[0] == 'disco1':
            self.disco =+1
        elif self.bow ==1:
            if self.neighbors.predict([self.data_from_sub])[0] == 'bow2':
                self.pub.publish('bow')
            else:
                self.bow = 0
        elif self.neighbors.predict([self.data_from_sub])[0] == 'bow1':
            self.bow =+1
        else:
            self.pub.publish(self.neighbors.predict([self.data_from_sub])[0])
        #print(self.neighbors.predict([self.data_from_sub]))

    def checking(self,gesture):
        if self.check == 5:
            self.gestures.values[gesture] =+1
            self.check = 0
            self.publishing()
        else:
            self.check =+1

    def publishing(self):
        [key for key,val in self.gestures.items() if val == max(self.gestures.values())]
        self.pub.publish(gesture)

    def run(self):
        r = rospy.Rate(10)
        self.training()
        while not rospy.is_shutdown():
            # self.close_points()
            self.machine_learning()
            r.sleep()

if __name__ == "__main__":
    #import doctest
    gest = Gestures([(1,2,3),(5,3,6)])
    gest.run()
    #doctest.testmod()
    #doctest.run_docstring_examples(Gestures.close_points, globals(),verbose = True)
