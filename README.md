# Interactive Robotics Laboratory
ST-R17 and UR5 interactive co-working dinosaur robotic arms
Website: https://olinrobotics.github.io/edwin/
Arm Manual: http://strobotics.com/manuals/R17%20manual.pdf

##Troubleshooting

Grouped by error message

*Encoder-stepper mismatch*
* Is the area around the K11R control box and robot arm clear?
* Turn the controller on/off. (Power-cycle the robot)

---

*rosrun irl arm_node.py is stuck at "in block_on_result"*
* Is the turn-key at the front of the controller set to warm?
* Is the light on the Tripp-Lite serial-usb converter blinking?
* Open a new terminal window and run the command again

---

*Controller refuses to turn on*
* Check the fuses. Two are located in the back of the controller, one is located on the power supply itself. (Where the power cord is plugged in)
