---
title: Olin Multirotor Projects
layout: template
filename: index
--- 

# What We Do
The [Olin Robotics Lab](http://olinrobotics.github.io) works on many projects involving multirotor aircraft equiped with pixhawk autopilots. The Multirotors repository contains all of our code for communicating with the Pixhawk, mainly using [Mavros](http://wiki.ros.org/mavros). We have a set of basic functions for sending missions to the pixhawk, changing its flight mode, and giving it pwm inputs.  This repository is also includes code for our specific software based projects.

## Our projects

### Landing on a fiducial
In an effort to establish a system in which drones are reliable research tools that require no previous piloting experience, we are working on an autonomous landing system in which drones will be able to return to a landing platform on a moving vehicle and land on as small of a platform as possible.  We are using AR codes on the platform and a downwards facing camera on the drone to accurately locate the drone relative to the target and land.

### Photogrametry
We are working on accurately dimensioning an object using a camera feed from a drone.  We are using the position of the drone and angle of the camera to find the distance between pixles in the image.

## Software Architecture
The Multirotors github repository is setup as a ROS package which contains basic code for using a multirotor equipped with a pixhawk, as well as code for our specific projects, separated into subdirecotories.

### Main directory:
**drone.py**
- Layer of control directly under the drone
- Relays control commands from other nodes to the drone
- Listens to info from drone
- Contains functions for requesting functions from the drone
  - RTL, land, arm disarm
- Inherits from Missions, (and by extension, Guided and Waypoints)

**Missions.py**
- Contains code for building mission files for the drone
  - easily add waypoints and select start and end behaviours
  - save mission to a file
- Contains functions for beginning, ending, and restarting missions
- Manages a boundery checker which supports boundaries consisting of convex polygons
- Inherits from Guided and Waypoints

**Guided.py**
- Manages guided mode
  - function for switching into guided mode
  - function for add guided waypoints
  - function for auto starting guided mode then cycling through a list of waypoints on sequential button presses
  - Inherits from Waypoints

**Waypoints.py**
- Builds waypoints from lat, lon with optional alt
- Contains functions to manage waypoints on drone
  - pushes new waypoints
  - resets mission
  - adds waypoints to end of current mission
  - clears waypoints

**joytest.py and keyboardtest.py**
- Listen to inputs and convert them to ros messages for drone.py
  - Joystick can control modes, arm, disarm, and control drone
  - Keyboard can do everything joystick can except actually control the drone

### mission_planning directory
This directory contains a mission planner type interface to work with our code.  It supports setting waypoints for both missions and guided mode, as well as setting boundaries for a bounded flight using a polygon.  Directions for how to use it are in its readme.

### fiducial_tracking directory
The goal of this project is to hit a button and have the drone go to a landing target, then see a fiducial on the target and autonomously land on it.

**fiducial_follower.py**
- contains code for finding a fiducial and controlint the drone to keep the fiducial centered
- Uses ar_pose to find targets
- Uses PID nodes to run control loops
- Uses GPS velocity to try to return to the target if it goes out of sight

**goto_target.py**
- listens to a USB gps outputing the `$GPGGA` NEMA message and directs the drone to that location using guided mode.

## Useful Information

- [Running Our Code](readme)