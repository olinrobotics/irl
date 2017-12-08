#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from irl.srv import arm_cmd

class ToolPicker():
    def __init__(self):
        rospy.init_node("ToolPicker", anonymous=True)
        # receive tool command from gui
        self.cmd_sub = rospy.Subscriber("/tool_cmd", String, self.command_callback)
        # receive image feed from the camera
        self.image_sub = rospy.Substriber("/usb_cam/image_raw", Image, self.image_callback)
        # publish to arm and speech behavior
        self.arm_pub = rospy.Publisher("/arm_cmd", String, queue_size=10)
        self.say_pub = rospy.Publisher("edwin_speech_cmd", String, queue_size=10)
        self.status_sub = rospy.Subscriber("/arm_status", String, self.status_callback)

        # check actuation status
        self.status = -1

        # initialize callback variables
        self.cmd = ""
        self.frame = None

        # x,y location will be programmed
        self.target_x = 0
        self.target_y = 0

        # z location will be hardcoded for each tool
        self.target_z = 0

    def command_callback(self, data):
        self.cmd = data.data

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
        # location to be determined for the arm to welcome player
        wel_x = 0
        wel_y = 0
        wel_z = 0

        msg = "data: move_to:: " + wel_x + ", " + wel_y + ", " + wel_z + ", 0"
        print("Sending: ", msg)
        self.request_cmd(msg)
        statement = "Hello! I am here to help. What tool do you want me to pick up?"
        self.say_pub.publish(statement)
        time.sleep(1)

    def find_tool(self):
        # run neural network to locate tool location
        pass

    def pick_up_tool(self):
        pass

    def run(self):
        welcome_player()

        # wait for user to imput command
        while self.cmd == "":
            pass

        self.find_tool()
        self.pick_up_tool()

if __name__ == "__main__":
    tp = ToolPicker()
    tp.run()
