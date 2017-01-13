---
title: How to Edwin
layout: template
filename: readme
--- 

### Getting Code to Run
1. Clone [github repository](https://github.com/olinrobotics/edwin) into your `catkin_ws/src` folder
  - if you don't have ROS indigo, install it with [these instructions](http://wiki.ros.org/indigo/Installation/Ubuntu) (use `ros-indigo-desktop-full`)
2. Install dependencies: 

  Basic processes:
  
    - copy
    - random
    - time
    - logging
    - datetime
    - pickle
    - os
    - sys
    - serial
    - re
    - shlex
    - threading
    - subprocess
    - struct
    
  Ros packages
  
    - std_msgs
    - common_msgs (sensor_msgs)
    - rospy
    - rospkg
    - roslib
    
  Mathematical processing:
  
    - math
    - numpy
    - scipy
    - operator
    - itertools
    
  Visual processing:
  
    - cv2
    - cv_bridge
    - Tkinter
    - csv
  
  Audial processing:
  
    - pyaudio
    - Queue
    - alsaaudio
    - audioop
    - wave
    - pocketsphinx
    - sphinxbase
    - espeak
    
3. *Under construction*

### Launch Files
[**`robot.launch`**](https://github.com/olinrobotics/edwin/blob/master/launch/robot.launch)
Fully operational launch of all demo systems.

Requirements:

  - Loads Edwin's brain (brain.py)
  - Loads idle behaviors (idle.py)
  - USB cam feed required (image_raw, camera_info)
  - Arm must be on (arm_node.py, arm_behaviors.py, arm_draw.py)

[**`robot_minimal.launch`**](https://github.com/olinrobotics/edwin/blob/master/launch/robot_minimal.launch)
Mostly for quick behavior debugging. This is the minimum needed to "operate" Edwin. No camera.

Requirements:

  - Loads Edwin's brain (brain.py)
  - Arm must be on (arm_behaviors.py)
  - Kinect must be on and connected.

[**`robot_sight.launch`**](https://github.com/olinrobotics/edwin/blob/master/launch/robot_sight.launch)
Meant for testing only. This launch file only loads camera and Kinect scripts; the arm does not boot up.

Requirements:

  - Kinect must be on and connected.
  - USB cam feed required (image_raw, camera_info).

*This Github page is currently under construction. Last edited on 1/12/17.*
