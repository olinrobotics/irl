import rospkg
import rospy

class Features:
    def __init__(self):
        # defines file paths
        rospack = rospkg.RosPack()
        PACKAGE_PATH = rospack.get_path("edwin")
        self.faces_folder_path = PACKAGE_PATH + '/params'
