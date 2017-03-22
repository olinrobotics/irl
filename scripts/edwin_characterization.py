#!/usr/bin/env python
import rospkg
import rospy
import math
import time

import csv
from std_msgs.msg import String
from edwin.msg import Bones
from sklearn.neighbors import NearestNeighbors

#with open('file.csv', newline='') as csvfile:
#     spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
#     for row in spamreader:
#         print(', '.join(row))


with open('file.csv', 'rU') as file:
    table = [row for row in csv.reader(file)]
print(table)

with open('skeleton.csv', 'rU') as file:
    label= [col['A'] for col in csv.reader(file)]
    training_data = [col['E':'AE'] for col in csv.reader(file)]



neighbors = KNeighborsClassifier(n_neighbors=7,algorithm='ball_tree')
neighbors.fit( training_data, 'labels')
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
        rospy.Subscriber('/skeleton', String, self.data_callback)
        self.pub = rospy.Publisher('skeleton_points', Bones, queue_size=10)
        time.sleep(2)
        #pub = rospy.Publisher('something', std_msgs.msg.String, queue_size=10)
        #recieving data in forms of a list of tuples

    def data_callback(self,data):
        data.h.x
        data.h.y
        data.h.z
        data.n.x
        data.n.y
        data.n.z
        data.t.x
        data.t.y
        data.t.z
        data.rs.x
        data.rs.y
        data.rs.z
        data.re.x
        data.re.y
        data.re.z
        data.rh.x
        data.rh.y
        data.rh.z
        data.ls.x
        data.ls.y
        data.ls.z
        data.le.x
        data.le.y
        data.le.z
        data.lh.x
        data.lh.y
        data.lh.z
        data.rp.x
        data.rp.y
        data.rp.z
        data.rk.x
        data.rk.y
        data.rk.z
        data.rf.x
        data.rf.y
        data.rf.z
        data.lp.x
        data.lp.y
        data.lp.z
        data.lk.x
        data.lk.y
        data.lf.y
        data.lk.z
        data.lf.x
        data.lf.z
        print(data.data)
        self.pub.publish("Gesture detection is running")

    def body_parts(self):
        head = self.skeleton[0]
        left_hand = self.skeleton[1]
        right_hand = self.skeleton[2]
        left_shoulder = self.skeleton[3]
        right_shoulder = self.skeleton[4]
        left_elbow = self.skeleton[5]
        right_elbow = self.skeleton[6]
        chest = self.skeleton[7]

    def close_points(self):
        distance_x =abs(self.skeleton[0][0]-self.skeleton[1][0])
        distance_y = abs(self.skeleton[0][1]-self.skeleton[1][1])
        distance_btwn_xy= math.sqrt((distance_x)**2 + (distance_y)**2)
        distance_z = abs(self.skeleton[0][2]-self.skeleton[1][2])
        distance_btwn_points= math.sqrt((distance_btwn_xy)**2+(distance_z)**2)
        if distance_btwn_points <5:
            self.pub.publish("True")
            return True
        else:
            self.pub.publish("False")
            return False

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # self.close_points()
            r.sleep()

if __name__ == "__main__":
    #import doctest
    gest = Gestures([(1,2,3),(5,3,6)])
    gest.run()
    #doctest.testmod()
    #doctest.run_docstring_examples(Gestures.close_points, globals(),verbose = True)
