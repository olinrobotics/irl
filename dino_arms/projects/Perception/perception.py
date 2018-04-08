"""
By Khang Vu & Sherrie Shen, 2018
Last Modified Mar 22, 2018

The script ...

Dependencies:
- realsense_ros_camera

To use:
- Open Terminal and run the code below:

roslaunch realsense_ros_camera rs_rgbd.launch

TODO:
- Hand Detection
- Fix hole problem
- Make paper grid
- Color detection to get the pixel of the paper grid
- From that, get two vectors on the table and make another function to find_height_angle
"""

import numpy as np

import cv2
import rospy
import sensor_msgs.point_cloud2 as pc2
from irl.msg import Cube, Structure
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image, PointCloud2
import transformation
import localization


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
        rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback, queue_size=10)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback, queue_size=10)
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.pointcloud_callback, queue_size=10)
        self.publisher = rospy.Publisher("perception", Structure, queue_size=10)
        self.cam = CameraType(camera_type, width, height)
        self.cube_size = cube_size
        self.r = rospy.Rate(10)
        self.rgb_data = self.depth_data = self.point_cloud = self.angle = self.height = None
        self._coords = [None] * self.cam.IMAGE_HEIGHT * self.cam.IMAGE_WIDTH

    def rgb_callback(self, data):
        try:
            self.rgb_data = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

    def depth_callback(self, data):
        try:
            self.depth_data = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print e

    def pointcloud_callback(self, data):
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
        gen = pc2.read_points(self.point_cloud, field_names=("x", "y", "z"))
        for i, p in enumerate(gen):
            self._coords[i] = self.get_xyz_numpy(p)
        return np.asarray(self._coords)

    def get_transformed_coords(self):
        """
        Get the current transformed point cloud coordinates
        :return: numpy array of 3D transformed coordinates; if there's no transformed coordinates, return original ones
        """
        self.find_height_angle()
        if self.is_not_nan(self.height):
            return np.asarray(transformation.transformPointCloud(self._coords, self.angle, self.height))
        print "No angle found, returns original coordinates"
        return np.asarray(self._coords)

    def get_coord_from_pixel(self, pixel):
        """
        Get the coordinate of a pixel of an image
        :param pixel: pixel of an image
        :return: untransformed 3D coordinate of that pixel
        """
        while self.point_cloud is None:
            print "No point cloud found"

        row, col = pixel
        if row < 0 or col < 0 or row >= self.cam.IMAGE_HEIGHT or col >= self.cam.IMAGE_WIDTH:
            print "Row {} and Col {} are invalid".format(row, col)
            return

        points_gen = pc2.read_points(self.point_cloud, field_names=("x", "y", "z"))
        for i, p in enumerate(points_gen):
            if i == self.rowcol_to_i(row, col):
                return self.get_xyz_numpy(p)

    def get_coords_from_pixels(self, pixels):
        """
        Get coordinates of pixels of an image
        :param pixels: pixels of the image
        :return: untransformed 3D coordinates of these pixels
        """
        while self.point_cloud is None:
            print "No point cloud found"

        list = []
        for i, pixel in enumerate(pixels):
            row, col = pixel
            if row < 0 or col < 0 or row >= self.cam.IMAGE_HEIGHT or col >= self.cam.IMAGE_WIDTH:
                print "Row {} and Col {} are invalid".format(row, col)
                continue
            list.append(self.rowcol_to_i(row, col))

        coords = []
        count = 0
        points_gen = pc2.read_points(self.point_cloud, field_names=("x", "y", "z"))
        for i, p in enumerate(points_gen):
            if i in list:
                coord = self.get_xyz_numpy(p)
                count += 1
                if self.is_not_nan(coord[0]):
                    coords.append(coord)
                if count == len(list):
                    break
        return np.asarray(coords)

    def find_height_angle(self):
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
        ab = self.distance(a, b)
        oa = self.distance(o, a)
        ob = self.distance(o, b)

        # Find the angle using cosine law
        angle = 90 - np.degrees(np.arccos((ab ** 2 + oa ** 2 - ob ** 2) / (2 * oa * ab)))

        # Find height of the camera
        height = np.cos(np.radians(angle)) * oa

        # If not nan, update angle and height
        if self.is_not_nan(height):
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
    def distance(a, b):
        """
        :param a: array
        :param b: array
        :return: distance between two points in 2D
        """
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    @staticmethod
    def is_not_nan(a):
        return a is not None and not np.isnan(a)

    def publish(self, cubes):
        structure = Structure()
        for cube in cubes:
            structure.building.append(Cube(height=0, connections=0, x=cube[0], y=cube[2], z=cube[1]))
        self.publisher.publish(structure)

    def is_hand(self):
        # TODO: write function
        return False

    def get_structure(self):
        if self.is_hand():
            return
        coords = self.get_transformed_coords()
        print "original", self._coords[self.rowcol_to_i(self.cam.MID_ROW, self.cam.MID_COL)]
        print "transformed", coords[self.rowcol_to_i(self.cam.MID_ROW, self.cam.MID_COL)]
        cubes = localization.cube_localization(coords, self.cube_size)
        print cubes, len(cubes), "cubes"
        self.publish(cubes)

    """ TESTING FUNCTIONS """

    def test_transform_coords(self):
        coords = self.get_transformed_coords()
        print "original", self._coords[self.rowcol_to_i(self.cam.MID_ROW, self.cam.MID_COL)]
        print "transformed", coords[self.rowcol_to_i(self.cam.MID_ROW, self.cam.MID_COL)]
        np.savetxt('coords_8.txt', coords, fmt='%f')
        cubes = localization.cube_localization(coords, self.cube_size)
        print cubes
        print len(cubes), "cubes"
        import plot
        plot.plot_cube2d(cubes)


if __name__ == '__main__':
    perception = Perception(cube_size=localization.CUBE_SIZE_SMALL)
    perception.show_rgbd()
    # while True:
    #     angle, height = perception.find_height_angle()
    #     print angle, height
    # if angle:
    #     break
    # perception.get_structure()
    # cubes = [[0, 0, 2], [3, 4, 3]]
    r = rospy.Rate(10)
    while True:
        r.sleep()
        perception.get_structure()
