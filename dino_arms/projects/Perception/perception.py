#!/usr/bin/env python

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
- More testing
- Modify publisher/subscriber
- Extend code to work with the big world
"""

import numpy as np
import random

import cv2
import rospy
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridgeError, CvBridge
from irl.msg import Real_Cube, Real_Structure
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool

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
        rospy.Subscriber('/controller/status', Bool, self._building_status, queue_size=10)
        self.publisher = rospy.Publisher("perception", Real_Structure, queue_size=10)
        self.cam = CameraType(camera_type, width, height)
        self.cube_size = cube_size
        self.rgb_data = self.depth_data = self.point_cloud = None
        self.angle = self.height = None
        self.cubes = None
        self._coords = [None] * self.cam.IMAGE_HEIGHT * self.cam.IMAGE_WIDTH

    def _rgb_callback(self, data):
        try:
            self.rgb_data = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print e

    def _depth_callback(self, data):
        try:
            self.depth_data = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print e

    def _pointcloud_callback(self, data):
        self.point_cloud = data

    def _building_status(self, data):
        if data.data is False:
            self.get_structure()

    def show_rgb(self):
        """
        Show rgb video
        :return: None
        """
        while self.rgb_data is None:
            pass

        while not rospy.is_shutdown():
            rospy.Rate(10).sleep()
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
            rospy.Rate(10).sleep()
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
            rospy.Rate(10).sleep()
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
        return self._coords

    def get_transformed_coords(self):
        """
        Get the current transformed point cloud coordinates
        :return: numpy array of 3D transformed coordinates; if there's no transformed coordinates, keep searching for one
        """
        self.find_height_angle()
        while not self._is_not_nan(self.angle) and not self._is_not_nan(self.height):
            self.find_height_angle()
            print "No angle found. Keep searching for an angle!"
        print "height", self.height, "angle", self.angle
        return transformation.transformPointCloud(self._coords, self.angle, self.height)

    def find_paper_coords(self, show_video=True):
        """
        Find all white pixels of the paper and get the corresponding coordinates
        :param show_video: True to show the processed image
        :return:
        """
        blur = cv2.GaussianBlur(self.rgb_data, (5, 5), 0)
        lower_white = np.array([140, 140, 140])
        upper_white = np.array([255, 255, 255])
        mask = cv2.inRange(blur, lower_white, upper_white)

        # Erosion
        kernel = np.ones((5, 5), np.uint8)
        erosion_mask = cv2.erode(mask, kernel, iterations=2)

        paper_coords = []
        for row, item in enumerate(erosion_mask):
            for col, value in enumerate(item):
                if value == 255:
                    i = self.rowcol_to_i(row, col)
                    if self._coords[i] is not None and self._is_not_nan(self._coords[i][0]):
                        paper_coords.append(self._coords[i])
        if show_video:
            cv2.imshow('image', self.rgb_data)
            cv2.imshow('mask', erosion_mask)
            cv2.waitKey(0)

        return paper_coords

    def find_height_angle(self):
        """
        Find the angle and height of the camera
        :return: angle (degrees), height
        """
        paper_coords = self.find_paper_coords()
        if len(paper_coords) < 3:
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

        heights_angles = []
        for _ in range(10):
            i1, i2, i3 = random.sample(range(0, len(paper_coords)), 3)
            points = [paper_coords[i1], paper_coords[i2], paper_coords[i3]]
            while self._is_linear_dependent(points):
                i3 = random.randint(0, len(paper_coords))
                points = [paper_coords[i1], paper_coords[i2], paper_coords[i3]]

            # Find 10 heights and angles of the camera
            heights_angles.append([find_angle(points), find_height(points)])
        heights_angles.sort(key=lambda x: x[0])
        self.angle, self.height = heights_angles[5]
        return self.angle, self.height

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
        """
        Check three points on a surface and see if they're linear dependent (a.k.a on the same line)
        :param points: 3 points
        :return: True if linear dependent
        :rtype: bool
        """
        v1, v2 = points[0] - points[1], points[1] - points[2]
        angle = np.arccos(np.matmul(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
        angle = np.degrees(angle)
        if abs(angle) < 15 or abs(angle) > 165:
            print "Linear dependent. Finding a different point"
            return True
        return False

    @staticmethod
    def _distance(a, b):
        """
        :param a: array
        :param b: array
        :return: distance between two points in 2D
        :rtype: float
        """
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    @staticmethod
    def _is_not_nan(a):
        return a is not None and not np.isnan(a)

    def _publish(self):
        if self.cubes is None:
            return
        else:
            print self.cubes, len(self.cubes), "cubes"
        structure = Real_Structure()
        for cube in self.cubes:
            structure.building.append(Real_Cube(x=cube[0], y=cube[2], z=cube[1]))
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
            rospy.Rate(20).sleep()
            if not self._has_hand():
                self.get_pointcloud_coords()
                print "Original", self._coords[self.rowcol_to_i(self.cam.MID_ROW, self.cam.MID_COL)]
                coords = self.get_transformed_coords()
                print "Transformed", coords[self.rowcol_to_i(self.cam.MID_ROW, self.cam.MID_COL)]
                cubes = localization.cube_localization(coords, self.cube_size)
                if coords is not None and self._coords is not None and len(cubes) != 0:
                    if self.cubes is None or abs(len(self.cubes) - len(cubes)) <= 8:
                        self.cubes = cubes
                        self._publish()
                    else:
                        self.get_structure()
                else:
                    self.get_structure()
            else:
                self.get_structure()
        else:
            self.get_structure()

    def run(self):
        print "Perception running"
        self.show_rgb()
        self.get_structure()
        while not rospy.is_shutdown():
            try:
                rospy.Rate(10).sleep()
            except KeyboardInterrupt:
                break


if __name__ == '__main__':
    perception = Perception(cube_size=localization.CUBE_SIZE_SMALL)
    perception.run()
