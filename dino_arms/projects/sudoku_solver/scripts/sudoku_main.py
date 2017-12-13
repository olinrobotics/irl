import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridgeError, CvBridge
from irl.msg import Edwin_Shape
from irl.srv import arm_cmd
from sensor_msgs.msg import Image
from std_msgs.msg import String

import get_sudoku


class SudokuMain(object):
    """
    Master class of the game Sudoku
    """

    def __init__(self, n=4):
        # init ROS nodes
        self.route_string = 'R_sudoku_center'
        self.z_offset = 75
        rospy.init_node('sudoku_gamemaster', anonymous=True)

        # init ROS subscribers to camera and status
        rospy.Subscriber('arm_cmd_status', String, self.status_callback, queue_size=10)
        rospy.Subscriber('writing_status', String, self.writing_status_callback, queue_size=20)
        self.image_sub = rospy.Subscriber('usb_cam/image_raw', Image, self.img_callback)

        self.write_pub = rospy.Publisher('/write_cmd', Edwin_Shape, queue_size=10)

        # For the image
        self.bridge = CvBridge()

        # Edwin's status: 0 = busy, 1 = free
        self.status = 0
        self.writing_status = 1

        # Video frame
        self.frame = None

        # x, y, z positions of Edwin
        self.x = 0
        self.y = 0
        self.z = 0

        # Sudoku size, either 4 or 9
        self.n = n

        # Captured image from the camera
        self.sudoku_image = None

        # Sudoku object
        self.sudoku = None

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

    def writing_status_callback(self, data):
        """
        Get status from arm_write node
        :param data: status callback
        :return: None
        """
        print "writing status callback", data.data
        if data.data == "writing":
            print "Busy"
            self.writing_status = 0
        elif data.data == "done":
            print "Free"
            self.writing_status = 1

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
        while self.status == 0 or self.writing_status == 0:
            if self.status == 0:
                print "Moving"
            else:
                print "Writing"
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
        self.move_xyz(x=0, y=3400, z=4700)
        self.move_head(hand_value=3350, wrist_value=4020)

    def move_to_center_route(self):
        """
        Move edwin to the center position where it can take a good picture
        Using routes
        :return: None
        """
        self.route_move(self.route_string)

    def capture_piture(self):
        """
        Capture picture from usb_cam and pass it to self.sudoku_image
        :return: None
        """
        r = rospy.Rate(10)
        while self.frame is None:
            r.sleep()
            pass

        self.sudoku_image = self.frame.astype(np.uint8)
        cv2.imshow('Image', self.sudoku_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def capture_video(self):
        r = rospy.Rate(10)
        while self.frame is None:
            r.sleep()
            pass

        while self.frame is not None:
            r.sleep()
            cv2.imshow('Image', self.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def write_number(self, row, col, number):
        """
        Move the arm to (row, col) and write a number
        :return: None
        """
        x, y, z = self.get_coordinates(row, col)
        self.move_xyz(x, y, z + 300)
        data = Edwin_Shape(x=x, y=y, z=z - self.z_offset, shape=str(number))
        self.write_pub.publish(data)
        self.check_completion()
        self.move_xyz(x, y, z + 300)

    def write_numbers(self):
        """
        Write solution on the Sudoku board
        :return: None
        """
        solution = self.sudoku.solution
        for cell in solution:
            row, col, number = cell.get_rc_num()
            print "row = %i, col = %i, num = %i" % (row, col, number)
            self.write_number(row, col, number)
            if not self.continue_or_not():
                self.move_to_center()
                break

    def test_write_numbers(self):
        for i in range(1, 2):
            for j in range(4):
                print "Writing", i, j
                self.write_number(i, j, 8)
                # if not self.continue_or_not():
                #     break

    def continue_or_not(self):
        """
        Function asks the users if they want to continue the program.
        There is a question asking if the users want to change the z_offset
        :return: True to continue; False otherwise
        """
        answer = raw_input("Do you want me to continue (yes/no)? ").lower()
        if "y" in answer:

            answer = raw_input(
                "Current offset is %s. Do you want to change the z_offset (yes/no)? " % self.z_offset).lower()
            if "y" in answer:
                while True:
                    answer = raw_input("New z_offset = ")
                    try:
                        self.z_offset = int(answer)
                        print self.z_offset
                        break
                    except ValueError:
                        # Handle the exception
                        print 'Invalid value. Please enter an integer.'
            print "Continue"
            return True

        print "Stopped"
        return False

    def run(self):
        """
        Main function that runs everything
        :return: None
        """
        # self.capture_video()
        self.move_to_center_route()
        self.move_to_center()
        self.capture_piture()
        self.sudoku = get_sudoku.from_image(im=self.sudoku_image, n=self.n)
        if self.continue_or_not():
            self.sudoku.print_sudoku()
            self.write_numbers()
            self.move_to_center()

    def get_coordinates(self, row, col):
        """
        Get coordinates x, y, z from row, col
        :param row: 0 to 3
        :param col: 0 to 3
        :return: x, y, z
        """
        if row == 0 and col == 0:
            x = -1500
            y = 6500
            z = -780

        elif row == 0 and col == 1:
            x = -400
            y = 6500
            z = -770

        elif row == 0 and col == 2:
            x = 800
            y = 6500
            z = -770

        elif row == 0 and col == 3:
            x = 1900
            y = 6500
            z = -770

        elif row == 1 and col == 0:
            x = -1500
            y = 5400
            z = -770

        elif row == 1 and col == 1:
            x = -450
            y = 5400
            z = -770

        elif row == 1 and col == 2:
            x = 700
            y = 5400
            z = -768

        elif row == 1 and col == 3:
            x = 1900
            y = 5400
            z = -774

        elif row == 2 and col == 0:
            x = -1500
            y = 4200
            z = -758

        elif row == 2 and col == 1:
            x = -400
            y = 4300
            z = -755

        elif row == 2 and col == 2:
            x = 700
            y = 4300
            z = -753

        elif row == 2 and col == 3:
            x = 1900
            y = 4400
            z = -763

        elif row == 3 and col == 0:
            x = -1500
            y = 3200
            z = -735

        elif row == 3 and col == 1:
            x = -450
            y = 3200
            z = -738

        elif row == 3 and col == 2:
            x = 700
            y = 3200
            z = -744

        elif row == 3 and col == 3:
            x = 1900
            y = 3200
            z = -750

        else:
            x = 0
            y = 3400
            z = 4700

        return x, y, z


if __name__ == '__main__':
    sudoku_game = SudokuMain(n=4)
    sudoku_game.run()
