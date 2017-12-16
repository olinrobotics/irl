"""
By Khang Vu, Cassandra Overney, Enmo Ren & Emma Pan, 2017
Last modified Dec 13, 2017

This script is the master script of the Set Game
Currently works for a 3x4 Set board
"""
import time

import rospy
from cv_bridge import CvBridgeError, CvBridge
from irl.srv import arm_cmd
from sensor_msgs.msg import Image
from std_msgs.msg import String

from Turn import *
from opencv import *


class SetMain(object):
    """
    Master class of the game Set
    """

    def __init__(self):
        # init ROS nodes
        self.route_string = 'R_set_center'
        self.z_offset = 75
        rospy.init_node('set_gamemaster', anonymous=True)

        # init ROS subscribers to camera and status
        rospy.Subscriber('arm_cmd_status', String, self.status_callback, queue_size=10)
        rospy.Subscriber('usb_cam/image_raw', Image, self.img_callback)

        self.behavior_pub = rospy.Publisher('behaviors_cmd', String, queue_size=10)

        # For the image
        self.bridge = CvBridge()

        # Edwin's status: 0 = busy, 1 = free
        self.status = 0

        # Video frame
        self.frame = None

        # x, y, z positions of Edwin
        self.x = 0
        self.y = 0
        self.z = 0

        # Captured image from the camera
        self.set_image = None

        # Result: a set
        self.result = []

    def status_callback(self, data):
        """
        Get status from arm_node
        :param data: status callback
        :return: None
        """
        print "Arm status callback", data.data
        if data.data == "busy" or data.data == "error":
            print "Busy"
            self.status = 0
        elif data.data == "free":
            print "Free"
            self.status = 1

    def img_callback(self, data):
        """
        Get image from usb camera
        :param data: image
        :return: None
        """
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

    def request_cmd(self, cmd):
        rospy.wait_for_service('arm_cmd', timeout=15)
        cmd_fnc = rospy.ServiceProxy('arm_cmd', arm_cmd)
        print "I have requested the command"

        try:
            cmd_fnc(cmd)
            print "Command done"

        except rospy.ServiceException, e:
            print ("Service call failed: %s" % e)

        self.check_completion()

    def check_completion(self, duration=1.0):
        """
        Makes sure that actions run in order by waiting for response from service
        """
        time.sleep(duration)
        r = rospy.Rate(10)
        while self.status == 0:
            r.sleep()
            pass

    def route_move(self, route_string):
        msg = "data: run_route:: " + route_string
        print ("sending: ", msg)
        self.request_cmd(msg)

    def move_xyz(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        msg = "data: move_to:: %i, %i, %i, %i" % (self.x, self.y, self.z, 0)
        print ("Sending", msg)
        self.request_cmd(msg)

    def move_wrist(self, value):
        msg = "data: rotate_wrist:: " + str(value)
        print ("sending: ", msg)
        self.request_cmd(msg)

    def move_hand(self, value):
        msg = "data: rotate_hand:: " + str(value)
        print ("sending: ", msg)
        self.request_cmd(msg)

    def behave_move(self, behavior_string):
        msg = behavior_string
        print ("sending: ", msg)
        self.behavior_pub.publish(msg)

    def move_head(self, hand_value=None, wrist_value=None):
        """
        Always move hand first, wrist second
        :param hand_value:
        :param wrist_value:
        :return: None
        """
        self.move_hand(hand_value)
        self.move_wrist(wrist_value)

    def move_to_center(self):
        """
        Move edwin to the center position where it can take a good picture
        :return: None
        """
        self.move_xyz(x=0, y=3500, z=2000)
        self.move_head(hand_value=1900, wrist_value=2500)

    def move_to_center_route(self):
        """
        Move edwin to the center position where it can take a good picture
        Using routes
        :return: None
        """
        self.route_move(self.route_string)

    def capture_piture(self):
        """
        Capture picture from usb_cam and pass it to self.set_image
        :return: None
        """
        r = rospy.Rate(10)
        while self.frame is None:
            r.sleep()
            pass

        self.set_image = self.frame.astype(np.uint8)
        cv2.imshow('Image', self.set_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def capture_video(self):
        """
        Capture video from usb_cam
        :return: None
        """
        r = rospy.Rate(10)
        while self.frame is None:
            r.sleep()
            pass

        while self.frame is not None:
            r.sleep()
            cv2.imshow('Image', self.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def pout_behavior(self):
        """
        Make Edwin do sad behavior to show that there is no Set
        :return:
        """
        self.behave_move("pout")

    def pick_card(self, row, col):
        """
        Move the arm to (row, col) and pick up the card
        :return: None
        """
        print "row = %i, col = %i" % (row, col)
        x, y, z = self.get_coordinates_3_by_4(row, col)
        self.move_xyz(x, y, z - 400)

    def pick_cards(self):
        """
        Pick a set (3 cards) from the board
        :return: None
        """
        for set_card in self.result:
            row, col = set_card.coord
            self.pick_card(row, col)
            if not self.continue_or_not():
                self.move_to_center()
                break
            # self.move_to_stack()
            # if not self.continue_or_not():
            #     self.move_to_center()
            #     break

    def move_to_stack(self):
        """
        Move a card to a stack out of the board
        :return:
        """
        self.move_xyz(x=0, y=3000, z=2700)

    def test_pick_cards(self):
        """
        Test function to pick cards on the 3 x 4 board
        :return: None
        """
        for i in range(4):
            for j in range(3):
                self.pick_card(i, j)

    def continue_or_not(self):
        """
        Function asks the users if they want to continue the program.
        There is a question asking if the users want to change the z_offset
        :return: True to continue; False otherwise
        """
        answer = raw_input("Do you want me to continue (yes/no)? ").lower()
        if "y" in answer:
            print "Continue"
            return True

        print "Stopped"
        return False

    def play_again(self):
        """
        Function asks the users if they want to play the game again.
        :return: True to continue; False otherwise
        """
        answer = raw_input("Do you want to play again (yes/no)? ").lower()
        if "y" in answer:
            print "Re-play"
            return True

        print "Game ended"
        return False

    def run(self):
        """
        Main function that runs everything
        :return: None
        """
        # self.capture_video()
        # self.move_to_center_route()
        while True:
            self.move_to_center()
            self.capture_piture()
            if not self.continue_or_not():
                if self.play_again():
                    continue
                else:
                    self.behave_move("done_game")
                    break

            all_cards = find_matches(self.set_image)
            turn = Turn(all_cards)
            self.result = turn.find_set()
            turn.print_card_array(self.result)

            cv2.imshow('Image', self.set_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            if not self.continue_or_not():
                if self.play_again():
                    continue
                else:
                    self.behave_move("done_game")
                    break

            if not self.result:
                self.pout_behavior()
                self.move_to_center()
            else:
                self.pick_cards()
            self.move_to_center()
            if not self.play_again():
                self.behave_move("done_game")
                break

    def get_coordinates_3_by_4(self, row, col):
        """
        Get coordinates x, y, z from row, col
        Only applicable for 3 x 4 board
        :param row: 0 to 2
        :param col: 0 to 3
        :return: x, y, z
        """
        if row == 0 and col == 0:
            x = -700
            y = 5800
            z = 0

        elif row == 0 and col == 1:
            x = 400
            y = 5800
            z = 0

        elif row == 0 and col == 2:
            x = 1400
            y = 5800
            z = 0

        elif row == 1 and col == 0:
            x = -700
            y = 5000
            z = 0

        elif row == 1 and col == 1:
            #
            x = 400
            y = 5000
            z = 0

        elif row == 1 and col == 2:
            x = 1400
            y = 5000
            z = 0

        elif row == 2 and col == 0:
            x = -700
            y = 4300
            z = 0

        elif row == 2 and col == 1:
            x = 400
            y = 4300
            z = 0

        elif row == 2 and col == 2:
            x = 1400
            y = 4300
            z = 0

        elif row == 3 and col == 0:
            x = -700
            y = 3600
            z = 0

        elif row == 3 and col == 1:
            x = 400
            y = 3600
            z = 0

        elif row == 3 and col == 2:
            x = 1400
            y = 3600
            z = 0

        else:
            x = 0
            y = 3400
            z = 4700

        return x, y, z


if __name__ == '__main__':
    set_game = SetMain()
    set_game.run()
