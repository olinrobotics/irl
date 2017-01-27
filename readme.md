---
title: How to Edwin
layout: template
filename: readme
---

### Installation

[Click here for installation instructions.](install_edwin)

### Startup (Minimal)

Run Edwin with the minimal amount of code necessary to actuate him.

  1. Run [roscore.](http://wiki.ros.org/roscore)
  2. Press Ctrl-Alt-T to start a new tab.
  3. Plug the USB cord into a port on your computer. You may require a USB serial converter (pictured below) to connect to the robot.
  
  ![Connector](images/USBConnect.jpeg)
    
  4. Turn Edwin on by flipping the power button on his electronics box to "|". (Pictured below)
  
  ![Switch](images/PowerSwitch.jpeg)

  5. In your new terminal, run `roslaunch edwin robot_minimal.launch`
  6. To test functionality, run `rosrun edwin test_arm_pub.py`
  7. You should now be able to type positions in the terminal

*This Github page is currently under construction. Last edited on 1/27/17.*
