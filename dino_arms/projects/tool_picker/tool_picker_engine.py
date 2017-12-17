#!/usr/bin/env python

import rospy
import numpy as np
import time

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sift_finder import sift_finder

import cv2

class ToolPicker():
    def __init__(self):
        rospy.init_node("ToolPicker", anonymous=True)
        # receive tool command from gui
        self.cmd_sub = rospy.Subscriber("/tool_cmd", String, self.command_callback)
        # receive image feed from the camera
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        # publish to arm and speech behavior
        self.coordinates_pub = rospy.Publisher("/coordinates_cmd", String, queue_size=10)
        self.joints_pub = rospy.Publisher("/joints_cmd", String, queue_size=10)
        self.say_pub = rospy.Publisher("/edwin_speech_cmd", String, queue_size=10)
        self.status_sub = rospy.Subscriber("/arm_status", String, self.status_callback)
        self.bridge = CvBridge()

        # check actuation status
        self.status = -1

        # initialize callback variables
        self.prev_cmd = ""
        self.cmd = ""
        self.frame = None

        # x,y location will be programmed
        self.target_x = 0
        self.target_y = 0
        self.tool_list = ['Clamp','Cutter', 'Screwdriver','Wrench','Scissors','Piler']

        # z dictionary for each tool
        # Clamp: 544.58,  -72.15, -217.42, 2.5475, -2.2448, 0.3414
        # Cutter: 618.84, -67.60, -197.66, 2.5880, -2.2750, 0.4620
        # Scissor: 700.17, -76.01, -211.71, 2.5937, -2.3193, 0.5710
        # SvrewDriver: 583.5, 297.01, -180.30, 1.6286, -2.7966, 0.3216
        # Wrench: 621.14, 258.69, -182.75, 1.5584, -3.0017, 0.4328
        # Piler: 691.87, 256.99, -176.88, 1.9490, -2.7767, 0.5292

        # z location will be hardcoded for each tool
        # TODO: calibrate offset and Z position with new mounts
        self.z_dict = {'Clamp':0.5, 'Cutter':0.5, 'Screwdriver':0.5, 'Wrench':0.5, 'Scissors':0.5, 'Piler':0.5}
        self.xy_offset = {'Clamp':(0,0), 'Cutter':(0,0), 'Screwdriver':(0,0), 'Wrench':(0,0),'Scissors':(0,0),'Piler':(0,0)}

    def command_callback(self, data):
        self.cmd = data.data
        print(self.cmd)

    def image_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

    def status_callback(self, data):
        print("Arm status:", data.data)
        if data.data == "busy" or data.data == "error":
            self.status = 0
        elif data.data == "free":
            self.status = 1


    def welcome_player(self):
        # welcome joints angles: 112 -90 98.23 -188.78 0 0
        msg = "tp_welcome"
        print("Sending: ", msg)
        self.joints_pub.publish(msg)
        statement = "Hello! I am here to help. What tool do you want me to pick up?"
        self.say_pub.publish(statement)
        time.sleep(1)

    def find_tool(self, tool_name):
        # run neural network to locate tool location
        # Camera joints: 15.29 -91.65 -84.42 -86.02 90.21 0.33
        statement = "Okay! I am looking for the tools right now, give me a minute"
        self.say_pub.publish(statement)
        msg = "tp_camera"
        print("Sending: ", msg)
        self.joints_pub.publish(msg)
        time.sleep(3)

        [flag, coordinate_list] = sift_finder(self.frame, self.cmd)
        print(coordinate_list)
        if not flag:
            statement = "Sorry the tool you looked for wasn't on the table. Let's try another one!"
            self.say_pub.publish(statement)
            print(statement)
        else:
            self.pick_up_tool(coordinate_list)
            statement = "Ha I found your tool! Wanna try another one?"
            self.say_pub.publish(statement)
        time.sleep(1)

    def pick_up_tool(self, coordinate_list):
        # Paper corners:
        # Bottom right: .5 -0.10 0.49878
        # Top right: .75 -0.10 0.49878
        # Top left: .72 0.28 0.49878
        # Bottom left: .45 0.28 0.49878
        # frame size 480, 640
        robot_x = 0.75 - coordinate_list[1] / 480 * (0.75 - 0.45)
        robot_y = 0.28 - coordinate_list[0] / 640 * (0.28 + 0.1)
        robot_z = self.z_dict[self.cmd]
        msg = str(robot_x) + ' ' + str(robot_y) + ' ' + str(robot_z)
        print('Sending:' + msg)
        self.coordinates_pub.publish(msg)
        time.sleep(2)
        # # TODO: jaw actuation
        #
        # # Return to welcome pose after finishing
        msg = 'tp_welcome'
        self.joints_pub.publish(msg)
        time.sleep(1)

    def run(self):
        print("Tool picker engine running")
        while not rospy.is_shutdown():
            # wait for user to imput command
            while self.cmd == "":
                pass
            if self.cmd == "start":
                self.welcome_player()

            while self.cmd == "start" or self.cmd == "":
                pass

            self.find_tool(self.cmd)
            time.sleep(2)

            self.cmd = ""

if __name__ == "__main__":
    tp = ToolPicker()
    tp.run()
