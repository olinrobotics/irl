---
title: Code Info
layout: template
filename: readme
--- 

### Getting Code to Run
1. Clone [github repository](https://github.com/olinrobotics/Multirotors) into your `catkin_ws/src` folder
  - if you don't have ROS indigo, install it with [these instructions](http://wiki.ros.org/indigo/Installation/Ubuntu) (use `ros-indigo-desktop-full`)
2. Install dependencies:
* Tkinter (`sudo apt-get install python-tk`) - for mission planning gui
* Numpy [install instructions](http://www.scipy.org/install.html)
* ros_pid (`sudo apt-get install ros-indigo-pid`) - for fiducial tracking
* usb_cam (`sudo apt-get install ros-indigo-usb-cam`) - for fiducial tracking
* ar_pose (TODO, figure out install instructions) - for fiducial tracking

### Control of Drone
**Manual Override**
Flipping ch6 on the RC transmitter to a pwm value above 1500 will give all control of RC channels back to the RC transmitter regardless of what the code is doing.

**Keyboard Control**
* `<space>`: RTL
* `<enter>`: Land
* `r`: arm
* `t`: disarm
* `o`: auto
* `l`: Loiter
* `m`: Stabilize (think manual)
* `p`: start planner
TODO: implement arrows and wasd to control drone, with q and z as alt adjustments

**Joystick Control**
We have control systems implemented for a joystick, an xbox controller, and a flight sim controller
TODO: document how all of those controls work (for now ask someone or look in joytest.py)

### Launch Files
**`barebones.launch`**
Input arguments:
- `joy_port` (default: `js0`) - location of joystick (`/dev/input/<joy_port>`)
- `drone_port` (default: `ttyUSB0`) - location of drone (`/dev/<drone_port`)

Other requirements:
- Requres a 900MHz radio plugged into a USB port to communicate with drone
- Will error without a joystick plugged into a USB port, but will still run
  - Change variable in `joytest.py` to set which joystick you are using

**`fiducial.launch`**
start code for landing on a fiducial
TODO: document better