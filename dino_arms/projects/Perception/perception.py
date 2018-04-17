"""
By Khang Vu & Sherrie Shen, 2018
Last Modified April 15, 2018

The script ...

Dependencies:
- realsense2_camera: https://github.com/intel-ros/realsense
- librealsense: https://github.com/IntelRealSense/librealsense
- rgbd.launch: https://github.com/ros-drivers/rgbd_launch.git

To use:
- Open Terminal and run the code below:

roslaunch realsense2_camera rs_rgbd.launch

TODO:
- Hand Detection
- Fix hole problem
- From that, get two vectors on the table and make another function to find_height_angle
- Ignore unreliable results
"""

import numpy as np
import random

import cv2
import rospy
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridgeError, CvBridge
from irl.msg import Real_Cube, Real_Structure
from sensor_msgs.msg import Image, PointCloud2
import color_detection
import localization
import skin_detector
import transformation


class CameraType:
    MID_ROW_D400 = 238
    MID_COL_D400 = 315
    DEPTH_UNIT_D400 = 1
    OFFSET_D400 = 0

    MID_ROW_SR300 = 218
    MID_COL_SR300 = 292
    DEPTH_UNIT_SR300 = 0.124986647279
    OFFSET_SR300 = 0.005

    def __init__(self, type="D400", width=640, height=480):
        if type is "D400":
            self.MID_ROW = CameraType.MID_ROW_D400
            self.MID_COL = CameraType.MID_COL_D400
            self.DEPTH_UNIT = CameraType.DEPTH_UNIT_D400
            self.OFFSET = CameraType.OFFSET_D400
        else:
            self.MID_ROW = CameraType.MID_ROW_SR300
            self.MID_COL = CameraType.MID_COL_SR300
            self.DEPTH_UNIT = CameraType.DEPTH_UNIT_SR300
            self.OFFSET = CameraType.OFFSET_SR300
        self.IMAGE_WIDTH = width
        self.IMAGE_HEIGHT = height


class Perception:
    def __init__(self, camera_type="D400", cube_size=localization.CUBE_SIZE_SMALL, width=640, height=480):
        self.bridge = CvBridge()
        rospy.init_node('depth_cam', anonymous=True)
        rospy.Subscriber('/camera/color/image_raw', Image, self._rgb_callback, queue_size=10)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self._depth_callback, queue_size=10)
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self._pointcloud_callback, queue_size=10)
        self.publisher = rospy.Publisher("perception", Real_Structure, queue_size=10)
        self.cam = CameraType(camera_type, width, height)
        self.cube_size = cube_size
        self.r = rospy.Rate(10)
        self.rgb_data = self.depth_data = self.point_cloud = None
        self.angle = self.height = None
        self.cubes = None
        self._coords = [None] * self.cam.IMAGE_HEIGHT * self.cam.IMAGE_WIDTH

    def _rgb_callback(self, data):
        try:
            self.rgb_data = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

    def _depth_callback(self, data):
        try:
            self.depth_data = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print e

    def _pointcloud_callback(self, data):
        self.point_cloud = data

    def show_rgb(self):
        """
        Show rgb video
        :return: None
        """
        while self.rgb_data is None:
            pass

        while not rospy.is_shutdown():
            self.r.sleep()
            cv2.imshow('RGB Image', self.rgb_data)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def show_depth(self):
        """
        Show depth video
        :return: None
        """
        while self.depth_data is None:
            pass

        while not rospy.is_shutdown():
            self.r.sleep()
            d = self.depth_data * self.cam.DEPTH_UNIT
            d = cv2.applyColorMap(d.astype(np.uint8), cv2.COLORMAP_RAINBOW)
            cv2.imshow('Depth Image', d)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def show_rgbd(self):
        """
        Show both rgb and depth video
        :return: None
        """
        while self.depth_data is None:
            pass

        while not rospy.is_shutdown():
            self.r.sleep()
            d = self.depth_data * self.cam.DEPTH_UNIT
            d = cv2.applyColorMap(d.astype(np.uint8), cv2.COLORMAP_RAINBOW)
            dc = np.concatenate((d, self.rgb_data), axis=1)
            cv2.imshow('RGB & Depth Image', dc)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def get_pointcloud_coords(self):
        """
        Get the current point cloud coordinates
        :return: numpy array of 3D coordinates
        """
        while self.point_cloud is None:
            print "No point cloud found"

        gen = pc2.read_points(self.point_cloud, field_names=("x", "y", "z"))
        for i, p in enumerate(gen):
            self._coords[i] = self.get_xyz_numpy(p)
        return np.asarray(self._coords)

    def get_transformed_coords(self):
        """
        Get the current transformed point cloud coordinates
        :return: numpy array of 3D transformed coordinates; if there's no transformed coordinates, return original ones
        """
        angle, height = self.find_height_angle()
        while not self._is_not_nan(angle):
            angle, height = self.find_height_angle()
            print "No angle found. Keep searching for an angle!"
        return np.asarray(transformation.transformPointCloud(self._coords, self.angle, self.height))

    def get_coords_from_pixels(self, pixels):
        """
        Get coordinates of pixels of an image, assuming we jave self._coords
        :param pixels: pixels of the image
        :return: untransformed 3D coordinates of these pixels
        """
        coords = []
        for p in pixels:
            row, col = p
            if row < 0 or col < 0 or row >= self.cam.IMAGE_HEIGHT or col >= self.cam.IMAGE_WIDTH:
                print "Row {} and Col {} are invalid".format(row, col)
                continue
            if self._coords[self.rowcol_to_i(row, col)] is not None and self._is_not_nan(
                    self._coords[self.rowcol_to_i(row, col)][0]):
                coords.append(self._coords[self.rowcol_to_i(row, col)])

        return np.asarray(coords)

    def find_height_angle(self):
        """
        Find the angle and height of the camera
        :return: angle (degrees), height
        """
        self.get_pointcloud_coords()
        table_pixels = color_detection.find_paper(image=self.rgb_data)
        table_coords = self.get_coords_from_pixels(table_pixels)
        if len(table_coords) < 3:
            return None, None

        def find_angle(points):
            v1, v2 = points[0] - points[1], points[1] - points[2]
            normal_vector = np.cross(v1, v2)
            zhat = np.asarray([0, 0, 1])
            angle = np.arccos(np.matmul(normal_vector, zhat) / (np.linalg.norm(normal_vector) * np.linalg.norm(zhat)))
            angle = np.degrees(angle)
            if angle > 90:
                angle = 180 - angle
            return angle

        def find_height(points):
            v1, v2 = points[0] - points[1], points[1] - points[2]
            normal_vector = np.cross(v1, v2)
            d = np.matmul(normal_vector, points[0])  # d in equation ax + by + cz = d
            origin = [0, 0, 0]
            height = abs(np.matmul(origin, normal_vector) - d) / np.linalg.norm(normal_vector)
            return height

        i1, i2, i3 = random.sample(range(0, len(table_coords)), 3)
        points = [table_coords[i1], table_coords[i2], table_coords[i3]]
        while self._is_linear_dependent(points):
            i3 = random.randint(0, len(table_coords))
            points = [table_coords[i1], table_coords[i2], table_coords[i3]]

        # Find angle of the camera
        angle = find_angle(points)
        # Find height of the camera
        height = find_height(points)
        # If not nan, update angle and height
        if self._is_not_nan(height):
            self.angle, self.height = angle, height
        return angle, height

    def find_height_angle_old(self):
        """
        Find the angle and height of the camera
        :return: angle (degrees), height
        """
        while self.point_cloud is None:
            pass

        # Get the current point cloud
        self.get_pointcloud_coords()

        # Find two points a and b on the ground. a also lies on Oz
        a = self._coords[self.rowcol_to_i(self.cam.MID_ROW, self.cam.MID_COL)]
        b = self._coords[self.rowcol_to_i(self.cam.MID_ROW + self.cam.MID_ROW / 4, self.cam.MID_COL)]
        o = np.asarray([0, 0, 0])

        # Find lengths of ab, oa, ob
        ab = self._distance(a, b)
        oa = self._distance(o, a)
        ob = self._distance(o, b)

        # Find the angle using cosine law
        angle = 90 - np.degrees(np.arccos((ab ** 2 + oa ** 2 - ob ** 2) / (2 * oa * ab)))

        # Find height of the camera
        height = np.cos(np.radians(angle)) * oa

        # If not nan, update angle and height
        if self._is_not_nan(height):
            self.angle, self.height = angle, height
        return angle, height

    def get_xyz(self, p):
        """
        Get coordinates from a point in point cloud
        :param p: point in point cloud
        :return: x, y, z in meter
        """
        x, y, z = p
        x = x * self.cam.DEPTH_UNIT + self.cam.OFFSET
        y = y * self.cam.DEPTH_UNIT + self.cam.OFFSET
        z = z * self.cam.DEPTH_UNIT + self.cam.OFFSET
        return x, y, z

    def get_xyz_numpy(self, p):
        """
        Get coordinates from a point in point cloud
        :param p: point in point cloud
        :return: x, y, z in meter as a numpy array
        """
        return np.asarray(self.get_xyz(p))

    def rowcol_to_i(self, row, col):
        return row * self.cam.IMAGE_WIDTH + col

    def i_to_rowcol(self, i):
        return i / self.cam.IMAGE_WIDTH, i % self.cam.IMAGE_WIDTH

    @staticmethod
    def _is_linear_dependent(points):
        v1, v2 = points[0] - points[1], points[1] - points[2]
        angle = np.arccos(np.matmul(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
        angle = np.degrees(angle)
        if abs(angle) < 15 or abs(angle) > 165:
            return True
        return False

    @staticmethod
    def _distance(a, b):
        """
        :param a: array
        :param b: array
        :return: distance between two points in 2D
        """
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    @staticmethod
    def _is_not_nan(a):
        return a is not None and not np.isnan(a)

    def _publish(self):
        if self.cubes is None:
            return
        structure = Real_Structure()
        for cube in self.cubes:
            structure.building.append(Real_Cube(x=cube[0], y=cube[2], z=cube[1]))
        print self.cubes, len(self.cubes), "cubes"
        self.publisher.publish(structure)

    def _has_hand(self):
        """
        Check if there is a human hand in the video stream
        :return: True (has hand) or False (no hand)
        """
        if self.rgb_data is None:
            return True
        return skin_detector.has_hand(self.rgb_data)

    def get_structure(self):
        if not self._has_hand():
            coords = self.get_transformed_coords()
            cubes = localization.cube_localization(coords, self.cube_size)
            print "original", self._coords[self.rowcol_to_i(self.cam.MID_ROW, self.cam.MID_COL)]
            print "transformed", coords[self.rowcol_to_i(self.cam.MID_ROW, self.cam.MID_COL)]
            if self.cubes is None or abs(len(self.cubes) - len(cubes)) <= 10:
                self.cubes = cubes
        self._publish()


if __name__ == '__main__':
    perception = Perception(cube_size=localization.CUBE_SIZE_SMALL)
    perception.show_rgbd()
    r = rospy.Rate(10)
    while True:
        r.sleep()
        perception.get_structure()
