
/*

This code is the Arbotix-M Robocontroller Servo Controller Node.
It is designed to use an external power supply to power the servo motor. 
It merely utilizes the ax12 library to control the MX-64 Servo using
SetPosition to turn the motor to a specified integer. It will also
use RosSerial as a Subscriber to take inputs from ROS to move the Servo.
*/


#include <ax12.h>
#include <BioloidController.h>

//ROS stuff
#include <ros.h>
#include <std_msgs/UInt16.h>


ros::NodeHandle nh;

void inc_message( const std_msgs::UInt16& setpos){
  SetPosition(1, setpos.data);
}


ros::Subscriber<std_msgs::UInt16> sub("motor", &inc_message);



void setup(){
  nh.getHardware() -> setBaud(9600);
  nh.initNode();
  nh.subscribe(sub);
  delay(1000);
  SetPosition(1,2050);

   
}

void loop(){
  nh.spinOnce();
  delay(1);
}  

