# Perception
## ```perception.py```
* This is the master script of cube detection and localization for a given structure. The cube size is  
This script initialize the depth camera.
* Get point clouds
* Call ```color_detection.py``` to perform tf transform using vectors defined by the point cloud of the table to  indicate the horizontal direction
* calculate the angle of camera tilted
* transform the point cloud such that the coordinate system of the camera aligns with that of the real world
* get xyz coordinates of each point
* call ```localization.py``` to check for cubes
* call ```skin_detection.py``` to check the presence of a hand



## ```localization.py```
This script performs localization of each cube, given a point cloud of the entire structure. The xyz coordinate system of the point cloud has been transformed to match the xyz coordinate system of the world to offset the effect of camera tilting at some nonzero angle. After the transformation, the y axis represents the up and down direction, which is the height of the structure, the z axis represents forward and backward direction, which is the depth of the structure with respect to the camera, and the x asis represents the left and right direction, which is width of the structure. Given the point cloud of the structure, we then execute the following steps to find the xyz coordinate of each cube with respect to the world:


##### Step1: ```reduced_coords(coords, cube_size)```
* Since the camera is mounted at some height, we limit the maximum height (y) of the point cloud and ignore any points whose y coordinates is greater than 5 times the height of each cube.
* To improve accuracy, we set a maximum z value (depth) and only focus on point clouds with z coordinate smaller than the threshold. Points with depth greater than this threshold are most likely to be noise.

##### Step2:
We then check the cube by height. We define the height of the top surface of the cubes at each floor level with some tolerance of error as the level. For example, the height (y) of the first level would equal to the height of the cube. The result is a slice of point clouds at each level where the thickness of the slice is the tolerance of error.

##### Step3: ```find_cubes_at_height(coords, height_level, cube_size)```
* For each slice, we compute the minimum x value, maximum x value, minimum z value and maximum z value of the structure and define a bounding box. The coordinates of the four corners of the bounding box are (min_x, max_z), (min_x, min_z), (max_x, max_z), (max_x, min_z).  
* We then sort the point cloud at each slice by depth and divide the slice into strips where the cubes in each strip have the same depth (z) with respect to the camera.
* Lastly, we scan each strip starting from the points with minimum x coordinates and progress to the right by increments of the cube size. The strip of point clouds is divided into multiple regions of square shape to allow for checking the presence of a cube.

##### Step4: ```check_cubes(coords, height_level, cube_size)```
* Give a confined region of square shape as defined in Step3, we first scanned the number of points in the point cloud. If the number of points in the region is less than threshold, we categorized that region as no cubes present. The threshold is currently defined to be
* If the region have number of points greater than the threshold, we then
* check the area formed by the point cloud
* check presence of a hole


## ```skin_detection.py```
* Given an array of RGB pixels, the presence of a hand is determined using a range of HSV pixel intensities that could be
considered as skin. A skinMask is then created to isolate the relevant pixels and has_hand() returns true if the count
reaches a certain threshold.

## ```color_detection.py```
* Perform paper detection using color detection of white given an image from the intel realsense camera and output the row and column of the paper.
