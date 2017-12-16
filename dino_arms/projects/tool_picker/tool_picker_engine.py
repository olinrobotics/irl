#!/usr/bin/env python

import rospy
import numpy as np
import time

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# from irl.srv import arm_cmd

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

        # z location will be hardcoded for each tool
        self.target_z = 0

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

    def check_completion(self):
        time.sleep(2)
        while self.status == 0:
            pass

    def request_cmd(self,cmd):
        rospy.wait_for_service("arm_cmd", timeout=15)
        cmd_fnc = rospy.ServiceProxy("arm_cmd", arm_cmd)
        print("Command requested")

        try:
            resp1 = cmd_fnc(cmd)
            print("Command finished")
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)
            self.arm_status.publish("error")

    def welcome_player(self):
        # welcome joints angles: 112 -90 98.23 -188.78 0 0
        msg = "tp_welcome"
        print("Sending: ", msg)
        self.joints_pub.publish(msg)
        statement = "Hello! I am here to help. What tool do you want me to pick up?"
        self.say_pub.publish(statement)
        time.sleep(1)

    def find_tool(self):
        # run neural network to locate tool location
        # Camera joints: 15.29 -91.65 -84.42 -86.02 90.21 0.33
        msg = "tp_camera"
        print("Sending: ", msg)
        self.joints_pub.publish(msg)
        statement = "Okay! I am looking for the tools right now, give me a minute"
        self.say_pub.publish(statement)
        time.sleep(1)

    def pick_up_tool(self):
        pass

    def run(self):
        print("Tool picker enginer running")
        while not rospy.is_shutdown():
            # wait for user to imput command
            while self.cmd == "":
                pass

            self.welcome_player()
            time.sleep(10)
            print('I am waiting')

            self.find_tool()
            time.sleep(20)
            # self.pick_up_tool()

if __name__ == "__main__":
    tp = ToolPicker()
    tp.run()
    cv2.destoryAllWindows()
