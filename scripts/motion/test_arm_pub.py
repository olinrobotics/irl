#!/usr/bin/env python
import rospy
import math
import numpy as np
from std_msgs.msg import String
import Tkinter as tk
import time

class ArmGui:
    def __init__(self, root):
        self.master = root
        self.x = tk.StringVar()
        self.y = tk.StringVar()
        self.z = tk.StringVar()
        self.pitch = tk.StringVar()

        self.init_control_fields()
        self.init_pub()

    def route_move(self, num):
        msg = "data: run_route:: " + str(self.route_s.get())
        print "sending: ", msg
        self.pub2.publish(msg)

    def behave_move(self, num):
        msg = "data: " + str(self.behav_s.get())
        print "sending: ", msg
        self.pub2.publish(msg)

    def xyz_move(self):
        msg = "data: move_to:: " + self.x.get() + ", " + self.y.get() + ", " + self.z.get() + ", " + self.pitch.get()
        print "sending; ", msg
        self.pub.publish(msg)

    def wrist_move(self, num):
        msg = "data: rotate_wrist:: " + str(self.wrist_s.get())
        print "sending: ", msg
        self.pub.publish(msg)

    def hand_move(self, num):
        msg = "data: rotate_hand:: " + str(self.hand_s.get())
        print "sending: ", msg
        self.pub.publish(msg)

    def elbow_move(self, num):
        msg = "data: rotate_elbow:: " + str(self.elbow_s.get())
        print "sending: ", msg
        self.pub.publish(msg)

    def shoulder_move(self, num):
        msg = "data: rotate_shoulder:: " + str(self.shoulder_s.get())
        print "sending: ", msg
        self.pub.publish(msg)

    def waist_move(self, num):
        msg = "data: rotate_waist:: " + str(self.waist_s.get())
        print "sending: ", msg
        self.pub.publish(msg)

    def speed_set(self, num):
        msg = "data: set_speed:: " + str(self.speed_s.get())
        print "sending: ", msg
        self.pub.publish(msg)

    def init_control_fields(self):
        self.f1 = tk.Frame(self.master)
        self.f1.pack(side=tk.TOP, fill=tk.X)

        self.f2 = tk.Frame(self.master)
        self.f2.pack(side=tk.TOP, fill=tk.X)

        self.f3 = tk.Frame(self.master)
        self.f3.pack(side=tk.TOP, fill=tk.X)

        self.f4 = tk.Frame(self.master)
        self.f4.pack(side=tk.TOP, fill=tk.X)

        self.f5 = tk.Frame(self.master)
        self.f5.pack(side=tk.TOP, fill=tk.X)

        self.f6 = tk.Frame(self.master)
        self.f6.pack(side=tk.TOP, fill=tk.X)

        self.f7 = tk.Frame(self.master)
        self.f7.pack(side=tk.TOP, fill=tk.X)

        self.f8 = tk.Frame(self.master)
        self.f8.pack(side=tk.TOP, fill=tk.X)

        self.f9 = tk.Frame(self.master)
        self.f9.pack(side=tk.TOP, fill=tk.X)

        self.route_s = tk.Entry(self.f7)
        self.route_s.bind('<Return>', self.route_move)
        tk.Label(self.f7, text="Route: ").pack(side=tk.LEFT)
        self.route_s.pack()

        tk.Label(self.f8, text="XYZ: ").pack(side=tk.LEFT)
        tk.Entry(self.f8, textvariable=self.x, width=5).pack(side=tk.LEFT)
        tk.Entry(self.f8, textvariable=self.y, width=5).pack(side=tk.LEFT)
        tk.Entry(self.f8, textvariable=self.z, width=5).pack(side=tk.LEFT)
        tk.Entry(self.f8, textvariable=self.pitch, width=5).pack(side=tk.LEFT)
        tk.Button(self.f8, text="Set", width=5, command=self.xyz_move).pack(side=tk.LEFT)

        self.behav_s = tk.Entry(self.f9)
        self.behav_s.bind('<Return>', self.behave_move)
        tk.Label(self.f9, text="Behavior: ").pack(side=tk.LEFT)
        self.behav_s.pack()

        self.wrist_s = tk.Entry(self.f1)
        self.wrist_s.bind('<Return>', self.wrist_move)
        tk.Label(self.f1, text="Wrist: ").pack(side=tk.LEFT)
        self.wrist_s.pack()

        self.hand_s = tk.Entry(self.f2)
        self.hand_s.bind('<Return>', self.hand_move)
        tk.Label(self.f2, text="Hand: ").pack(side=tk.LEFT)
        self.hand_s.pack()

        self.elbow_s = tk.Entry(self.f3)
        self.elbow_s.bind('<Return>', self.elbow_move)
        tk.Label(self.f3, text="Elbow: ").pack(side=tk.LEFT)
        self.elbow_s.pack()

        self.shoulder_s = tk.Entry(self.f4)
        self.shoulder_s.bind('<Return>', self.shoulder_move)
        tk.Label(self.f4, text="Shoulder: ").pack(side=tk.LEFT)
        self.shoulder_s.pack()

        self.waist_s = tk.Entry(self.f5)
        self.waist_s.bind('<Return>', self.waist_move)
        tk.Label(self.f5, text="Waist: ").pack(side=tk.LEFT)
        self.waist_s.pack()

        self.speed_s = tk.Entry(self.f6)
        self.speed_s.bind('<Return>', self.speed_set)
        tk.Label(self.f6, text="Speed: ").pack(side=tk.LEFT)
        self.speed_s.pack()

    def init_pub(self):
        rospy.init_node('arm_tester', anonymous=True)
        self.pub2 = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.pub = rospy.Publisher('arm_cmd', String, queue_size=10)

        # while not rospy.is_shutdown():
            # time.sleep(1)
            # msg = str(raw_input("Your command: "))
            # pub.publish(msg)

if __name__ == '__main__':
    root = tk.Tk()
    ArmGui(root)
    root.mainloop()
