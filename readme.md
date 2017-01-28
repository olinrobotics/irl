---
title: How to Edwin
layout: template
filename: readme
---

### Installation

[Click here for installation instructions.](install_edwin)

### Startup (Minimal)

Run Edwin with the minimal amount of code necessary to actuate him.

  1. Run [roscore](http://wiki.ros.org/roscore) in a new Terminal window.
  2. Press Ctrl-Alt-T to start a new Terminal tab.
  3. Plug the USB cord into a port on your computer. The connector (pictured below) uses a usb serial connector to connect to the robot.
  ![Connector](images/USBConnect.jpeg)

  4. Turn Edwin on by flipping the power button on his electronics box to "|". (Pictured below)
  ![Switch](images/PowerSwitch.jpeg)

  5. In your new terminal, cd into the scripts folder in the edwin folder
  6. run `rosrun edwin arm_node.py` to start the arm node
  7. In a new Terminal window, run `roslaunch edwin robot_minimal.launch` to start peripheral things
  8. (a) To test functionality, run `rosrun edwin test_arm_pub.py` in a new Terminal window\s\s
     (b) To run code, open a new Terminal window and run your code.\s\s

<p>If you followed 7a, you can now move Edwin to various positions. If you followed
7b, Edwin should execute your code.</p>

*This Github page is currently under construction. Last edited on 1/27/17.*
